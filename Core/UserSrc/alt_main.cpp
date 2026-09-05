#include <math.h>
#include <queue>
#include <numeric>

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

/* sEMG 表面肌电信号变量 */
float semg_pa0_raw = 0.0f; /*!< PA0 通道原始肌电值 */
float semg_pa2_raw = 0.0f; /*!< PA2 通道原始肌电值 */
float semg_pa0_filtered = 0.0f; /*!< PA0 通道滤波后肌电值 */
float semg_pa2_filtered = 0.0f; /*!< PA2 通道滤波后肌电值 */
float semg_pa0_envelope = 0.0f; /*!< PA0 通道肌电包络值 */
float semg_pa2_envelope = 0.0f; /*!< PA2 通道肌电包络值 */

uint32_t g_exo_run_us = 0;

/* ADC DMA 三通道数据缓冲区：PC4 (电源电压) + PA2 + PA0，位于 DTCM .dma_buf 段 */
__attribute__((section(".dma_buf"), aligned(32))) uint32_t g_adc_data[3] = {0};

/**
 * @brief 外骨骼主任务入口
 * @param argument  FreeRTOS 任务参数 (本项目中未使用)
 */
void AltMainTask(void *argument)
{
  /*----------------------------------------------------------------------
   * 启用 DWT (Data Watchpoint and Trace) 内核定时器
   * 为后续微秒级延时 (DWTDelayUs) 和性能测量提供精确时间基准
   *----------------------------------------------------------------------*/
  DWTInit();

  /* 复位 NRF54L15 扩展板无线模块; 暂不需复位, 让蓝牙不断 */
  // HAL_GPIO_WritePin(NRF54_RST_GPIO_Port, NRF54_RST_Pin, GPIO_PIN_RESET);
  // HAL_Delay(10);
  // HAL_GPIO_WritePin(NRF54_RST_GPIO_Port, NRF54_RST_Pin, GPIO_PIN_SET);
  // HAL_Delay(100);

  /*----------------------------------------------------------------------
   * 启动 ADC1 + DMA 三通道连续采样
   * 通道：PC4 (电池电压), PA2, PA0
   * 触发源：TIM2 溢出事件，每 1ms 自动触发一轮转换
   * 转换结果由 DMA 自动搬运至 g_adc_data[3]
   *----------------------------------------------------------------------*/
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_data, 3);

  /* 初始化三路 FDCAN */
  BspCanInit(&hfdcan1);
  BspCanInit(&hfdcan2);
  BspCanInit(&hfdcan3);

  /* 阻塞等待初始化 BMI088 六轴 IMU (躯干姿态传感器) */
  while (BMI088_init())
    ;

  /*----------------------------------------------------------------------
   * 构造外骨骼硬件描述符和核心对象
   *----------------------------------------------------------------------*/
  static ExoHardware exo_hw = {
    .motor_can1 = hfdcan2, /* 左侧电机 CAN */
    .motor_can2 = hfdcan1, /* 右侧电机 CAN */
    .sensor_can = hfdcan3, /* IMU CAN */
    .sensor_spi = hspi3, /* 传感器 SPI */
    .sensor_uart = huart8, /* 传感器串口 */
    .shell_uart = huart9, /* Shell 命令行串口 */
    .left_mag_encoder_uart = huart2, /* 左腿磁编码器串口 */
    .right_mag_encoder_uart = huart3, /* 右腿磁编码器串口 */
  };
  static ExoData exo_data;
  static Exo exo(exo_data, exo_hw);
  exo.Initialize();

  /* 使能两路 24 V电机电源 (POWER_24V_1 和 POWER_24V_2), 并延时 1 秒等待稳定 */
  HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin | POWER_24V_2_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);

  /* 启动 TIM2 定时器中断, 周期1ms */
  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
    if (g_timer2_flag == 1)
    {
      g_timer2_flag = 0;

      uint32_t start_tick = DWT_CYCCNT;
      exo.Run(); /* 外骨骼核心控制逻辑 (必须 1ms 内完成) */
      g_exo_run_us = DWTGetDeltaUs(start_tick);
    }
  }
}
