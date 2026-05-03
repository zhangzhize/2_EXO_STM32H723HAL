#include <math.h>
#include <queue>
#include <numeric>
#include <string.h>

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include "dwt.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "utils.h"
#include "shell.hpp"
#include "ws2812.h"
#include "bmi088_driver.h"
#include "status_led.hpp"
#include "fsr.hpp"
#include "exo.hpp"
#include "mahony.hpp"

/* 保存虚拟串口接收到的数据, 可以来自PC, 或其它类型设备 */
#include "usbd_cdc_if.h"
uint8_t cdc_rx_buffer[512] = {0};
uint8_t cdc_rx_flag = 0;

__attribute__((section(".dma_buf"), aligned(32))) uint32_t g_adc_data[3] = {0};

/* 外骨骼需要用到的外设句柄 */
ExoHardware g_exo_hw = {
    .motor_can = hfdcan1,
    .sensor_uart = huart8,
    .shell_uart = huart9,
    .left_mag_encoder_uart = huart2,
    .right_mag_encoder_uart = huart3,
};
/* 实例化外骨骼数据对象, 及控制外骨骼和处理外骨骼的对象 */
ExoData g_exo_data;
Exo *g_exo = new Exo(g_exo_data, g_exo_hw);

/* 用于将六/九轴IMU数据解算出姿态, 暂不需要 */
Mahony *gptr_mahony = new Mahony();

/* 属于用户真正的main函数 */
void AltMainTask(void *argument)
{
    /** 启用DWT内核定时器 */
    DWTInit(); 

    /** 复位扩展板上的两个NRF54L15, 拉低RST引脚一段时间 */
    HAL_GPIO_WritePin(NRF54_RST_GPIO_Port, NRF54_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(NRF54_RST_GPIO_Port, NRF54_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    /** 启动ADC+DMA: PC4(电源电压) + PA2 + PA0 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_data, 3);

    /* DMA接收双足无线传感数据, 波特率1000000 Bits/s */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_rx_buffer, UART8_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    /* DMA接收无线上位机控制命令, 波特率1000000 Bits/s */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uart9_rx_buffer, UART9_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);

    /* 初始化fdcan1 */
    BspCanInit();

    /* 蜂鸣器, 暂不启用 */
    // HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    // TIM12->CCR2 = TIM12->ARR / 2;
    // HAL_Delay(500);
    // TIM12->CCR2 = TIM12->ARR / 4;
    // HAL_Delay(500);
    // TIM12->CCR2 = 0;
    /* 板载IMU, 暂不启用 */
    // float gyro[3], accel[3], temperature;
    // float roll_deg, pitch_deg, yaw_deg;
    // while(BMI088_init());
    // gptr_mahony->Begin(200.0f);
    // while (1)
    // {
        // uint32_t start_us = GetSysTimeMs();
        // BMI088_read(gyro, accel, &temperature);
        // gptr_mahony->UpdateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
        // gptr_mahony->GetEulerAnglesDeg(roll_deg, pitch_deg, yaw_deg);
        // HAL_Delay(5);
    // }

    /* 给电机上电 */
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin|POWER_24V_2_Pin, GPIO_PIN_SET);
    HAL_Delay(1000); /* 稍微延迟一下等电机上电启动完毕 */
    g_exo->Initialize();
    HAL_Delay(1000); /* 稍微延迟一下 */

	/* 启动定时器 */
	HAL_TIM_Base_Start_IT(&htim2);  
	while (1)
	{   
        if (g_timer2_flag == 1)   /* 现在控制周期是1ms */
        {
            g_timer2_flag = 0;
            /* 外骨骼 */
            g_exo->Run();
        }
	}
}

/** 下面的函数都是测试函数, 不用关注 */

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

#include "arm_math.h"
/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
