/**
 * @file    bsp_can.h
 * @brief   BSP CAN 总线抽象层 —— 外骨骼 CAN 通信硬件抽象接口
 *
 * @details 本模块封装 STM32H723 FDCAN 外设，为上层应用（Exo 控制逻辑）提供
 *          统一的 CAN 收发接口。采用单例回调模式（全局 ctx + 函数指针），
 *          将 HAL 库的 FDCAN 中断回调转发到 Exo 实例的成员方法。
 *
 *          回调注册流程：
 *          1. 上层调用 BspCanRegisterRxCallback() 注册 ctx 和回调函数
 *          2. HAL 中断触发 HAL_FDCAN_RxFifo0Callback()
 *          3. 内部解析报文后调用已注册的回调，传入 ctx 实现上下文切换
 *
 *          使用限制：当前仅支持一个 CAN 实例注册回调（单例模式）。
 *
 * @note    STM32H723 的 FDCAN 支持经典 CAN 和 CAN FD 两种帧格式，
 *          本模块在发送时根据数据长度自动选择帧格式（>8 字节使用 CAN FD）。
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"

/* CAN 接收回调函数类型 —— 单例模式核心：将 HAL 中断数据转发到 Exo 对象上下文 */
typedef void (*BspCanRxCallback)(void *ctx, FDCAN_HandleTypeDef *hfdcan, uint32_t can_ext_id, const uint8_t *rx_data);

/* 初始化 FDCAN 过滤器和水印，启动外设并开启 RX FIFO0 消息到达中断 */
void BspCanInit(FDCAN_HandleTypeDef *hfdcan);
/* 发送 CAN 报文：根据 len 自动选择经典 CAN（<=8 字节）或 CAN FD 帧格式 */
void FDCanSendData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint32_t id_type, uint8_t *data, uint32_t len);
/* 注册 CAN 接收回调 —— Exo 实例将自己的 this 指针作为 ctx 传入，实现成员函数回调 */
void BspCanRegisterRxCallback(void *ctx, BspCanRxCallback cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
