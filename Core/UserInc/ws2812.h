/**
 * @file    ws2812.h
 * @brief   WS2812 RGB LED 驱动 —— 外骨骼状态指示灯光控制
 *
 * @details 
 * @note    
 */

#ifndef WS2812_H
#define WS2812_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/**
 * @brief  阻塞模式设置 WS2812 颜色
 * @param  red   红色分量 (0-255)
 * @param  green 绿色分量 (0-255)
 * @param  blue  蓝色分量 (0-255)
 *
 * @details 使用 HAL_SPI_Transmit() 阻塞发送，函数返回时 LED 已更新。
 *          适用于对时序要求不敏感的状态切换场景。
 */
void LightWs2812(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief  DMA 模式设置 WS2812 颜色
 * @param  red   红色分量 (0-255)
 * @param  green 绿色分量 (0-255)
 * @param  blue  蓝色分量 (0-255)
 *
 * @details 使用 HAL_SPI_Transmit_DMA() 异步发送，数据缓冲区位于
 *          .bdma_buf 内存段（通过 BDMA 可访问的特殊内存区域）。
 *          适用于高频率更新的场景（如呼吸灯动画），不阻塞主控制循环。
 */
void LightWs2812BDMA(uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif

#endif
