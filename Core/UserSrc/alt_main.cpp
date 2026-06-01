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

#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "utils.h"
#include "shell.hpp"
#include "ws2812.h"
#include "bmi088_driver.h"
#include "status_led.hpp"
#include "exo.hpp"
#include "mahony.hpp"

#include "semg.hpp"


/* USB 虚拟串口接收的数据及其标志位 */
#include "usbd_cdc_if.h"
uint8_t cdc_rx_buffer[512] = {0};
uint8_t cdc_rx_flag = 0;

float semg_pa0_raw = 0.0f;
float semg_pa2_raw = 0.0f;
float semg_pa0_filtered = 0.0f;
float semg_pa2_filtered = 0.0f;
float semg_pa0_envelope = 0.0f;
float semg_pa2_envelope = 0.0f;

__attribute__((section(".dma_buf"), aligned(32))) uint32_t g_adc_data[3] = {0};

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

    /** 启动ADC+DMA: PC4(电源电压) + PA2 + PA0. 由TIM2触发 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_data, 3);

    /* 初始化fdcan1 */
    BspCanInit(&hfdcan1);
    BspCanInit(&hfdcan3);

    /* 外骨骼躯干IMU节点依赖这个函数 */
    while(BMI088_init());

    static ExoHardware exo_hw = {
        .motor_can = hfdcan1,
        .dm_imu_can = hfdcan3,
        .sensor_spi = hspi3,
        .sensor_uart = huart8,
        .shell_uart = huart9,
        .left_mag_encoder_uart = huart2,
        .right_mag_encoder_uart = huart3,
    };
    static ExoData exo_data;
    static Exo exo(exo_data, exo_hw);

    /* 给电机上电 */
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin|POWER_24V_2_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);             /* 稍微延迟一下等电机上电启动完毕 */
    exo.Initialize();

    /* 启动定时器 */
    HAL_TIM_Base_Start_IT(&htim2);
    while (1)
    {
        if (g_timer2_flag == 1) /* 控制周期 1ms */
        {
            g_timer2_flag = 0;

            /* 外骨骼 */
            exo.Run();
        }
    }
}
