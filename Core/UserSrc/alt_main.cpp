#include <math.h>
#include <queue>
#include <numeric>
#include <string.h>

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"


#include "utils.h"
#include "vofa.hpp"
#include "ws2812.h"
#include "bmi088_driver.h"
#include "status_led.hpp"
#include "fsr.hpp"
#include "exo.hpp"

#include "pcap01.hpp"
#include "adg2128.hpp"

PCAP01 pcap01;
ADG2128 adg2128_row;
ADG2128 adg2128_col;

/* 用于离线调试 */
#include "usbd_cdc_if.h"
uint8_t cdc_rx_buffer[512] = {0};
uint8_t cdc_rx_flag = 0;

Vofa *gptr_vofa = new Vofa();
ExoData *gptr_exo_data = new ExoData();
Exo *gptr_exo = new Exo(gptr_exo_data);

uint32_t g_adc_data[3] = {0};

void CDCTask1();
void CDCTask2();
void CDCTask3();

void AltMainTask(void *argument)
{
    uint32_t loop_cnt = 0;

    /** 复位扩展板上的两个NRF54L15 */
//    HAL_GPIO_WritePin(NRF54_BRIDGE_RST_GPIO_Port, NRF54_BRIDGE_RST_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(NRF54_SENSOR_RST_GPIO_Port, NRF54_SENSOR_RST_Pin, GPIO_PIN_RESET);
//    HAL_Delay(10);
//    HAL_GPIO_WritePin(NRF54_BRIDGE_RST_GPIO_Port, NRF54_BRIDGE_RST_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(NRF54_SENSOR_RST_GPIO_Port, NRF54_SENSOR_RST_Pin, GPIO_PIN_SET);
//    HAL_Delay(100);

    /** 启动ADC+DMA: PC4(电源电压) + PA2 + PA0 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_data, 3);

    /* DMA接收双足无线传感数据, 波特率1000000 Bits/s */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_rx_buffer, UART8_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    /* DMA接收无线上位机控制命令, 波特率1000000 Bits/s */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uart9_rx_buffer, UART9_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);

    /* 蜂鸣器, 暂不启用 */
    // HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    // TIM12->CCR2 = TIM12->ARR / 2;
    // HAL_Delay(500);
    // TIM12->CCR2 = TIM12->ARR / 4;
    // HAL_Delay(500);
    // TIM12->CCR2 = 0;

    /* 初始化CAN */
    BspCanInit();
    // BspStdCanInit();
    // BspExtCanInit();
    
    /* 给电机上电 */
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin|POWER_24V_2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin|POWER_24V_2_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);

    /** #HACK: 在此修改外骨骼参数  */
//    gptr_exo->Initialize();

    /* CDC */
    // pcap01.begin(&hspi3, PCAP01_CS_GPIO_Port, PCAP01_CS_Pin);
    // adg2128_row.Begin(&hi2c1, 0, MUX_ROW_RST_GPIO_Port, MUX_ROW_RST_Pin);
    // adg2128_col.Begin(&hi2c1, 1, MUX_COL_RST_GPIO_Port, MUX_COL_RST_Pin);

	/* 启动定时器 */
	HAL_TIM_Base_Start_IT(&htim2);

    /* 板载IMU */
    // float gyro[3], accel[3], temp;
    // while(BMI088_init());

	while (1)
	{
		// CDCTask2();

        if (g_timer2_5ms_flag == 1)
        {
            g_timer2_5ms_flag = 0;

            /* 外骨骼 */
            // gptr_exo->Run();

            /* 通过nrf54(uart9)蓝牙上报数据 */
            gptr_vofa->ptr_vofa_data_[0] = loop_cnt++;
            gptr_vofa->ptr_vofa_data_[1] = GetSysTimeMs();
            gptr_vofa->ptr_vofa_data_[2] = gptr_exo->left_side_.heel_fsr_.raw_reading_;
            gptr_vofa->ptr_vofa_data_[3] = gptr_exo->left_side_.toe_fsr_.raw_reading_;
            gptr_vofa->ptr_vofa_data_[4] = gptr_exo->right_side_.heel_fsr_.raw_reading_;
            gptr_vofa->ptr_vofa_data_[5] = gptr_exo->right_side_.toe_fsr_.raw_reading_;
            gptr_vofa->ptr_vofa_data_[6] = gptr_exo_data->left_side_.foot_imu_.quat_i_;
            gptr_vofa->ptr_vofa_data_[7] = gptr_exo_data->left_side_.foot_imu_.quat_j_;
            gptr_vofa->ptr_vofa_data_[8] = gptr_exo_data->left_side_.foot_imu_.quat_k_;
            gptr_vofa->ptr_vofa_data_[9] = gptr_exo_data->left_side_.foot_imu_.quat_real_;
            gptr_vofa->ptr_vofa_data_[10] = gptr_exo_data->right_side_.foot_imu_.quat_i_;
            gptr_vofa->ptr_vofa_data_[11] = gptr_exo_data->right_side_.foot_imu_.quat_j_;
            gptr_vofa->ptr_vofa_data_[12] = gptr_exo_data->right_side_.foot_imu_.quat_k_;
            gptr_vofa->ptr_vofa_data_[13] = gptr_exo_data->right_side_.foot_imu_.quat_real_;
            gptr_vofa->ptr_vofa_data_[14] = gptr_exo_data->left_side_.ankle_plantarflexion_force_N_;
            gptr_vofa->ptr_vofa_data_[15] = gptr_exo_data->right_side_.ankle_plantarflexion_force_N_;
            gptr_vofa->SendOneFrame(16);
            continue;

            gptr_vofa->ptr_vofa_data_[0] = loop_cnt++;
            gptr_vofa->ptr_vofa_data_[1] = GetSysTimeMs();
            gptr_vofa->ptr_vofa_data_[2] = gptr_exo_data->left_side_.hip_joint_.pos_rad_;
            gptr_vofa->ptr_vofa_data_[3] = gptr_exo_data->right_side_.hip_joint_.pos_rad_;
            gptr_vofa->ptr_vofa_data_[4] = gptr_exo->left_side_.hip_joint_.motor_.ctrl_param_.tor_set_Nm_;
            gptr_vofa->ptr_vofa_data_[5] = gptr_exo->right_side_.hip_joint_.motor_.ctrl_param_.tor_set_Nm_;
            gptr_vofa->SendOneFrame(6);
            continue;

            gptr_vofa->ptr_vofa_data_[0] = loop_cnt++;
            gptr_vofa->ptr_vofa_data_[1] = GetSysTimeMs();
            gptr_vofa->ptr_vofa_data_[2] = gptr_exo_data->ao_left_event_cnt_;
            gptr_vofa->ptr_vofa_data_[3] = gptr_exo_data->ao_right_event_cnt_;
            gptr_vofa->ptr_vofa_data_[4] = gptr_exo_data->left_side_.percent_gait_ / 100.0f;
            gptr_vofa->ptr_vofa_data_[5] = gptr_exo_data->right_side_.percent_gait_ / 100.0f;
            gptr_vofa->ptr_vofa_data_[6] = gptr_exo->left_side_.heel_fsr_.raw_reading_;
            gptr_vofa->ptr_vofa_data_[7] = gptr_exo->right_side_.heel_fsr_.raw_reading_;
            gptr_vofa->ptr_vofa_data_[8] = gptr_exo->left_side_.heel_fsr_.calibrated_reading_;
            gptr_vofa->ptr_vofa_data_[9] = gptr_exo->right_side_.heel_fsr_.calibrated_reading_;
            gptr_vofa->ptr_vofa_data_[10] = gptr_exo_data->left_side_.heel_contact_state_;
            gptr_vofa->ptr_vofa_data_[11] = gptr_exo_data->right_side_.heel_contact_state_;
            gptr_vofa->ptr_vofa_data_[12] = gptr_exo_data->right_side_.knee_joint_.pos_rad_;
            gptr_vofa->ptr_vofa_data_[13] = gptr_exo->right_side_.knee_joint_.motor_.torque_forward_;
            gptr_vofa->ptr_vofa_data_[14] = gptr_exo_data->left_side_.ankle_joint_.pos_rad_;
            gptr_vofa->ptr_vofa_data_[15] = gptr_exo->left_side_.ankle_joint_.motor_.torque_forward_;
            gptr_vofa->ptr_vofa_data_[16] = gptr_exo->left_side_.heel_fsr_.calibration_refinement_max_;
            gptr_vofa->ptr_vofa_data_[17] = gptr_exo->right_side_.heel_fsr_.calibration_refinement_max_;
            gptr_vofa->ptr_vofa_data_[18] = gptr_exo->left_side_.heel_fsr_.calibration_refinement_min_;
            gptr_vofa->ptr_vofa_data_[19] = gptr_exo->right_side_.heel_fsr_.calibration_refinement_min_;
            gptr_vofa->SendOneFrame(20);
        }
	}
}

void CDCTask1()
{
    static bool is_first_run = true;
    
    if (is_first_run)
    {
        is_first_run = false;
        adg2128_row.SetRoute(1, 0, false);  /* 先全部接地 */
        adg2128_col.SetRoute(1, 0, false);
        adg2128_row.SetRoute(1, 1, false);
        adg2128_col.SetRoute(1, 1, false);
    }

    /** 第一个电容 */
    adg2128_row.UnsetRoute(1, 0, true);  /* 从地中断开 */
    adg2128_col.UnsetRoute(1, 0, true);
    adg2128_row.SetRoute(0, 0, false);   /* 并连接到PCAP01 */
    adg2128_col.SetRoute(0, 0, false);
    DelayUs(50);

    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[0] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

    adg2128_row.UnsetRoute(0, 0, true);   /* 从PCAP01中断开 */
    adg2128_col.UnsetRoute(0, 0, true);
    adg2128_row.SetRoute(1, 0, false);    /* 连接到地 */
    adg2128_col.SetRoute(1, 0, false);
    DelayUs(50);

    /** 第二个电容 */
    adg2128_row.UnsetRoute(1, 0, true);  /* 从地中断开 */
    adg2128_col.UnsetRoute(1, 1, true);
    adg2128_row.SetRoute(0, 0, false);   /* 并连接到PCAP01 */
    adg2128_col.SetRoute(0, 1, false);
    DelayUs(50);

    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[1] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

    adg2128_row.UnsetRoute(0, 0, true);   /* 从PCAP01中断开 */
    adg2128_col.UnsetRoute(0, 1, true);
    adg2128_row.SetRoute(1, 0, false);    /* 连接到地 */
    adg2128_col.SetRoute(1, 1, false);
    DelayUs(50);

    /** 第三个电容 */
    adg2128_row.UnsetRoute(1, 1, true);  /* 从地中断开 */
    adg2128_col.UnsetRoute(1, 0, true);
    adg2128_row.SetRoute(0, 1, false);   /* 并连接到PCAP01 */
    adg2128_col.SetRoute(0, 0, false);
    DelayUs(50);

    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[2] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

    adg2128_row.UnsetRoute(0, 1, true);   /* 从PCAP01中断开 */
    adg2128_col.UnsetRoute(0, 0, true);
    adg2128_row.SetRoute(1, 1, false);    /* 连接到地 */
    adg2128_col.SetRoute(1, 0, false);
    //    HAL_Delay(1);
    DelayUs(50);

    /** 第四个电容 */
    adg2128_row.UnsetRoute(1, 1, true);  /* 从地中断开 */
    adg2128_col.UnsetRoute(1, 1, true);
    adg2128_row.SetRoute(0, 1, false);   /* 并连接到PCAP01 */
    adg2128_col.SetRoute(0, 1, false);
    DelayUs(50);

    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[3] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

    adg2128_row.UnsetRoute(0, 1, true);   /* 从PCAP01中断开 */
    adg2128_col.UnsetRoute(0, 1, true);
    adg2128_row.SetRoute(1, 1, false);    /* 连接到地 */
    adg2128_col.SetRoute(1, 1, false);
    DelayUs(50);

    gptr_vofa->ptr_vofa_data_[0] = pcap01.applyLowPassFilter(0, gptr_vofa->ptr_vofa_data_[0]);
    gptr_vofa->ptr_vofa_data_[1] = pcap01.applyLowPassFilter(1, gptr_vofa->ptr_vofa_data_[1]);
    gptr_vofa->ptr_vofa_data_[2] = pcap01.applyLowPassFilter(2, gptr_vofa->ptr_vofa_data_[2]);
    gptr_vofa->ptr_vofa_data_[3] = pcap01.applyLowPassFilter(3, gptr_vofa->ptr_vofa_data_[3]);
    
    gptr_vofa->ptr_vofa_data_[0] *= PCAP01::KRefCap;
    gptr_vofa->ptr_vofa_data_[1] *= PCAP01::KRefCap;
    gptr_vofa->ptr_vofa_data_[2] *= PCAP01::KRefCap;
    gptr_vofa->ptr_vofa_data_[3] *= PCAP01::KRefCap;
    gptr_vofa->SendOneFrame(4);
}


void CDCTask2()
{

    /** 第一个电容 */
	adg2128_row.SetRoute(0, 0, false);   /* 并连接到PCAP01 */
	adg2128_col.SetRoute(0, 0, false);
	DelayUs(50);

    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[0] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

    adg2128_row.UnsetRoute(0, 0, false);   /* 从PCAP01中断开 */
    adg2128_col.UnsetRoute(0, 0, false);
    DelayUs(50);

    /** 第二个电容 */
	adg2128_row.SetRoute(0, 0, false);   /* 并连接到PCAP01 */
	adg2128_col.SetRoute(0, 1, false);
	DelayUs(50);

    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[1] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

	adg2128_row.UnsetRoute(0, 0, false);   /* 并连接到PCAP01 */
	adg2128_col.UnsetRoute(0, 1, false);
	DelayUs(50);

    /** 第三个电容 */

    adg2128_row.SetRoute(0, 1, false);   /* 并连接到PCAP01 */
    adg2128_col.SetRoute(0, 0, false);
    DelayUs(50);

    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[2] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

    adg2128_row.UnsetRoute(0, 1, false);   /* 并连接到PCAP01 */
    adg2128_col.UnsetRoute(0, 0, false);
    DelayUs(50);

    /** 第四个电容 */
	adg2128_row.SetRoute(0, 1, false);   /* 并连接到PCAP01 */
	adg2128_col.SetRoute(0, 1, false);
	DelayUs(50);

    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[3] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

	adg2128_row.UnsetRoute(0, 1, false);   /* 并连接到PCAP01 */
	adg2128_col.UnsetRoute(0, 1, false);
	DelayUs(50);

    gptr_vofa->ptr_vofa_data_[0] = pcap01.applyLowPassFilter(0, gptr_vofa->ptr_vofa_data_[0]);
    gptr_vofa->ptr_vofa_data_[1] = pcap01.applyLowPassFilter(1, gptr_vofa->ptr_vofa_data_[1]);
    gptr_vofa->ptr_vofa_data_[2] = pcap01.applyLowPassFilter(2, gptr_vofa->ptr_vofa_data_[2]);
    gptr_vofa->ptr_vofa_data_[3] = pcap01.applyLowPassFilter(3, gptr_vofa->ptr_vofa_data_[3]);
    gptr_vofa->ptr_vofa_data_[0] *= PCAP01::KRefCap;
    gptr_vofa->ptr_vofa_data_[1] *= PCAP01::KRefCap;
    gptr_vofa->ptr_vofa_data_[2] *= PCAP01::KRefCap;
    gptr_vofa->ptr_vofa_data_[3] *= PCAP01::KRefCap;
    gptr_vofa->SendOneFrame(4);
}

void CDCTask3()
{
    pcap01.sendOpcode(PCAP01::KOpcodeStartCDCMeas);
    while(g_pcap01_intn_state);
    g_pcap01_intn_state = 1;
    gptr_vofa->ptr_vofa_data_[0] = pcap01.readRegister(PCAP01::KC1DivC0RegAddr, PCAP01::KFractionalBits);

    gptr_vofa->ptr_vofa_data_[0] = pcap01.applyLowPassFilter(0, gptr_vofa->ptr_vofa_data_[0]);
    gptr_vofa->ptr_vofa_data_[0] *= PCAP01::KRefCap;
    gptr_vofa->SendOneFrame(1);
    HAL_Delay(5);
}
