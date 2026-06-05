/**
 * @file    bsp_usart.h
 * @brief   BSP UART 抽象层 —— 外骨骼串口通信硬件抽象接口
 *
 * @details 封装 STM32H723 UART 外设，为上层（Exo 控制逻辑）提供 UART 收发与
 *          调试打印（printf 重定向）的硬件抽象。采用单例回调模式：
 *          - BspUsartRegisterRxCallback() 注册接收回调（ctx + 函数指针）
 *          - BspUsartRegisterErrorCallback() 注册错误回调
 *          - HAL 库的中断回调（HAL_UARTEx_RxEventCallback / HAL_UART_ErrorCallback）
 *            被本模块重写，将事件转发到已注册的回调
 *
 *          printf 重定向策略：
 *          - GCC 环境：重写 _write() 通过 USB CDC 发送
 *          - AC6/ARMCC 环境：重写 fputc() 通过 USB CDC 发送
 *          - 保留 UART 发送代码（注释），可按需切换
 *
 * @note    DMA 接收模式：上层配合 DMA 空闲中断实现不定长数据接收，
 *          HAL_UARTEx_RxEventCallback 在 DMA 半满/全满时触发。
 */

#ifndef BSP_USART_H
#define BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

/* UART 接收事件回调类型 —— HAL_UARTEx_RxEventCallback 的 BSP 层转发 */
typedef void (*BspUartRxCallback)(void *ctx, UART_HandleTypeDef *huart, uint16_t data_size);
/* UART 错误回调类型 —— HAL_UART_ErrorCallback 的 BSP 层转发 */
typedef void (*BspUartErrorCallback)(void *ctx, UART_HandleTypeDef *huart);

/* 注册 UART 接收回调（DMA 空闲中断触发时调用） */
void BspUsartRegisterRxCallback(void *ctx, BspUartRxCallback cb);
/* 注册 UART 错误回调 */
void BspUsartRegisterErrorCallback(void *ctx, BspUartErrorCallback cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USART_H */
