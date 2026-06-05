/**
 * @file    bsp_gpio.h
 * @brief   BSP GPIO 外部中断抽象层 —— 外骨骼 GPIO 中断硬件抽象接口
 *
 * @details 封装 STM32H723 GPIO 外部中断（EXTI），为上层提供统一的 GPIO 中断
 *          回调机制。采用单例回调模式：
 *          - BspGpioRegisterExtiCallback() 注册外部中断回调（ctx + 函数指针）
 *          - HAL_GPIO_EXTI_Callback() 被本模块重写，将中断事件转发到 Exo 实例
 *
 *          外骨骼系统中的典型应用场景：
 *          - 限位开关中断
 *          - 按键/急停按钮中断
 *          - 外部传感器数据就绪 (DRDY) 引脚中断
 *
 * @note    使用限制：当前仅支持一个 EXTI 回调实例（单例模式）。
 *          多个引脚的 EXTI 通过 GPIO_Pin 参数区分，由上层根据引脚号分发处理。
 */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"

/* GPIO 外部中断回调类型 —— HAL_GPIO_EXTI_Callback 的 BSP 层转发 */
typedef void (*BspGpioExtiCallback)(void *ctx, uint16_t GPIO_Pin);

/* 注册 GPIO 外部中断回调 */
void BspGpioRegisterExtiCallback(void *ctx, BspGpioExtiCallback cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
