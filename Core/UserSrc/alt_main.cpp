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

/* USB 虚拟串口 (CDC) 接收缓冲区及数据就绪标志 */
#include "usbd_cdc_if.h"
uint8_t cdc_rx_buffer[512] = {0}; /*!< CDC 接收缓冲区 */
float cdc_rx_floats[128] = {0.0f}; /*!< CDC 接收的浮点数数据 (512字节 / 4字节每float = 128) */
volatile uint16_t cdc_rx_len = 0; /*!< CDC 最近一次接收长度 */
volatile uint8_t cdc_rx_flag = 0; /*!< CDC 数据就绪标志 (非0表示有新数据) */

/* 本地复制缓冲区：放到文件作用域以避免在任务栈上分配大数组 */
static uint8_t rx_copy[512] = {0};
static uint16_t rx_len = 0;

/* sEMG 表面肌电信号变量 */
float semg_pa0_raw = 0.0f; /*!< PA0 通道原始肌电值 */
float semg_pa2_raw = 0.0f; /*!< PA2 通道原始肌电值 */
float semg_pa0_filtered = 0.0f; /*!< PA0 通道滤波后肌电值 */
float semg_pa2_filtered = 0.0f; /*!< PA2 通道滤波后肌电值 */
float semg_pa0_envelope = 0.0f; /*!< PA0 通道肌电包络值 */
float semg_pa2_envelope = 0.0f; /*!< PA2 通道肌电包络值 */

/* ADC DMA 三通道数据缓冲区：PC4 (电源电压) + PA2 + PA0，位于 DTCM .dma_buf 段 */
__attribute__((section(".dma_buf"), aligned(32))) uint32_t g_adc_data[3] = {0};

static void SendCdcReplayAck(void);
static bool DecodeCdcFloatFrame(const uint8_t *rx, uint16_t rx_len, uint32_t *seq, float *values, uint8_t *float_count, uint8_t max_values);

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
   * 通道：PC4 (电池电压), PA2 (肌电右), PA0 (肌电左)
   * 触发源：TIM2 溢出事件，每 1ms 自动触发一轮转换
   * 转换结果由 DMA 自动搬运至 g_adc_data[3]
   *----------------------------------------------------------------------*/
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_data, 3);

  /*----------------------------------------------------------------------
   * 初始化两路 FDCAN
   * hfdcan1: 电机 CAN 总线 (与左右腿电机通信)
   * hfdcan3: IMU 传感器 CAN 总线 (躯干姿态传感器)
   *----------------------------------------------------------------------*/
  BspCanInit(&hfdcan1);
  BspCanInit(&hfdcan3);

  /*----------------------------------------------------------------------
   * 初始化 BMI088 六轴 IMU (躯干姿态传感器)
   * 阻塞等待直至初始化成功，IMU 是姿态估计的关键传感器，必须就绪
   *----------------------------------------------------------------------*/
  while (BMI088_init())
    ;

  /*----------------------------------------------------------------------
   * 构造外骨骼硬件描述符和核心对象
   *----------------------------------------------------------------------*/
  static ExoHardware exo_hw = {
    .motor_can = hfdcan1, /* 电机 CAN */
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

  /*----------------------------------------------------------------------
   * 给 24V 电机电源上电
   * 同时使能两路 24V 电源 (POWER_24V_1 和 POWER_24V_2)
   * 延时 1 秒等待电机驱动器内部电容充电和初始化完成
   *----------------------------------------------------------------------*/
  HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin | POWER_24V_2_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);

  /*----------------------------------------------------------------------
   * 启动 TIM2 1kHz 定时器中断
   * 之后每 1ms 触发一次中断，在 ISR 中置 g_timer2_flag = 1
   *----------------------------------------------------------------------*/
  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
    if (g_timer2_flag == 1)
    {
      g_timer2_flag = 0;

      exo.Run(); /* 外骨骼核心控制逻辑 (~1ms 内完成) */
    }

    // if (cdc_rx_flag)
    if (false)
    {
      __disable_irq();
      rx_len = cdc_rx_len;
      if (rx_len > sizeof(rx_copy)) rx_len = sizeof(rx_copy);
      memcpy(rx_copy, cdc_rx_buffer, rx_len);
      cdc_rx_len = 0;
      cdc_rx_flag = 0;
      __enable_irq();

      uint32_t seq = 0;
      uint8_t float_count = 0;
      bool decoded = DecodeCdcFloatFrame(rx_copy, rx_len, &seq, cdc_rx_floats, &float_count, (uint8_t)DMA_UNION_BUF_SIZE_FLOATS);
      bool replay_loaded = false;

      if (decoded)
      {
        // replay_loaded = DebugInjectSlopeTelemetryFrame(exo_data, seq, cdc_rx_floats, float_count); /* (已删除, 需重新写) 将接收到的 CDC 浮点数据注入到系统中, 用于调试 */
        if (replay_loaded)
        {
          exo.Run();
        }
      }
      SendCdcReplayAck();
    }
  }
}

static void SendCdcReplayAck(void)
{
  uint8_t ack = (uint8_t)'R';
  for (uint8_t retry = 0; retry < 10u; retry++)
  {
    if (CDC_Transmit_HS(&ack, 1) == 0u)
    {
      return;
    }
    DWTDelayUs(100);
  }
}

static bool DecodeCdcFloatFrame(const uint8_t *rx, uint16_t rx_len, uint32_t *seq, float *values, uint8_t *float_count, uint8_t max_values)
{
  if (rx == nullptr || seq == nullptr || float_count == nullptr || values == nullptr) return false;
  if (rx_len < 5u) return false;

  memcpy(seq, &rx[0], sizeof(*seq));
  *float_count = rx[4];
  uint16_t expected_len = 5u + 4u * (uint16_t)(*float_count);
  if (*float_count == 0u || *float_count > max_values || rx_len < expected_len) return false;

  memcpy(values, &rx[5], (uint16_t)(*float_count) * sizeof(float));
  return true;
}
