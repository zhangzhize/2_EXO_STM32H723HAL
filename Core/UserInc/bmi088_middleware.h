/**
 * @file    bmi088_middleware.h
 * @brief   BMI088 中间件接口 —— 平台适配层（SPI 通信 + GPIO 片选 + 延时）
 *
 * @details 
 * @note    
 */
#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "stdint.h"

/* 启用 SPI 通信模式 */
#define BMI088_USE_SPI
//#define BMI088_USE_IIC

/* 中间件初始化：GPIO 初始化（由用户实现，通常为空，引脚由 CubeMX 配置） */
extern void BMI088_GPIO_init(void);
/* 中间件初始化：通信接口初始化（由用户实现，通常为空，SPI 由 CubeMX 配置） */
extern void BMI088_com_init(void);
/* 毫秒级延时 */
extern void BMI088_delay_ms(uint16_t ms);
/* 微秒级延时 */
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
/* 拉低加速度计 CS 片选信号 —— 选中加速度计 */
extern void BMI088_ACCEL_NS_L(void);
/* 拉高加速度计 CS 片选信号 —— 释放加速度计 */
extern void BMI088_ACCEL_NS_H(void);

/* 拉低陀螺仪 CS 片选信号 —— 选中陀螺仪 */
extern void BMI088_GYRO_NS_L(void);
/* 拉高陀螺仪 CS 片选信号 —— 释放陀螺仪 */
extern void BMI088_GYRO_NS_H(void);

/**
 * @brief  SPI 单字节全双工读写
 * @param  reg  要发送的字节（寄存器地址或数据/dummy）
 * @return 同时接收到的字节
 *
 * @details 底层调用 HAL_SPI_TransmitReceive() 完成一次全双工 SPI 传输
 */
extern uint8_t BMI088_read_write_byte(uint8_t reg);

/* added by zzz —— 高效突发读取 */
/**
 * @brief  加速度计 SPI 突发读取
 * @param  reg  起始寄存器地址
 * @param  buf  接收数据缓冲区
 * @param  len  要读取的有效字节数（不含地址和 dummy 字节）
 *
 * @details 一次 SPI 传输完成：发送地址 + 1 字节额外 dummy + 接收 len 字节数据。
 *          总传输长度 = len + 2，有效数据从 rx_buf[2] 开始。
 *          限制：len 不能超过 30（tx_buf/rx_buf 最大 32 字节）。
 */
extern void BMI088_accel_burst_read(uint8_t reg, uint8_t *buf, uint8_t len);
/**
 * @brief  陀螺仪 SPI 突发读取
 * @param  reg  起始寄存器地址
 * @param  buf  接收数据缓冲区
 * @param  len  要读取的有效字节数（不含地址字节）
 *
 * @details 一次 SPI 传输完成：发送地址 + 接收 len 字节数据。
 *          总传输长度 = len + 1，有效数据从 rx_buf[1] 开始。
 *          限制：len 不能超过 31。
 */
extern void BMI088_gyro_burst_read(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)

#endif

#endif
