/**
 * @file    bsp_spi.h
 * @brief   BSP SPI 总线抽象层 —— 外骨骼 SPI 通信硬件抽象接口
 *
 * @details 封装 STM32H723 SPI 外设，为上层传感器驱动（BMI088 IMU、WS2812 LED）
 *          提供统一的 SPI 错误处理抽象。采用单例回调模式：
 *          - BspSpiRegisterErrorCallback() 注册错误回调
 *          - HAL_SPI_ErrorCallback() 被本模块重写，转发 SPI 总线错误到上层
 *
 *          SPI 使用场景：
 *          - SPI2：连接 BMI088 加速度计和陀螺仪（双片选 CS）
 *          - SPI6：连接 WS2812 LED 灯带（SPI 位带技术模拟 WS2812 时序）
 *
 * @note    SPI 的 TxRx 完成回调 HAL_SPI_TxRxCpltCallback 已定义但为空体，
 *          保留给需要传输完成通知的场景使用。
 */

#ifndef BSP_SPI_H
#define BSP_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

/* SPI 错误回调函数类型 —— 将 HAL SPI 错误事件转发到 Exo 上下文 */
typedef void (*BspSpiErrorCallback)(void *ctx, SPI_HandleTypeDef *hspi);

/* 注册 SPI 错误回调 */
void BspSpiRegisterErrorCallback(void *ctx, BspSpiErrorCallback cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
