/**
 * @file    ws2812.c
 * @brief   WS2812 RGB LED 驱动实现 —— SPI 位带技术 + DMA/阻塞双模式
 *
 * @details 
 */

#include "ws2812.h"
#include "spi.h"
#include <string.h>

/* WS2812 0 码 SPI 编码 —— 约 25% 占空比 */
#define WS2812_LOW_LEVEL    (0xC0)
/* WS2812 1 码 SPI 编码 —— 约 50% 占空比 */
#define WS2812_HIGH_LEVEL   (0xF0)

/* WS2812 使用的 SPI 总线单元 —— SPI6 */
#define WS2812_SPI_UNIT   hspi6

/**
 * @brief  阻塞模式发送 1 颗 WS2812 LED 颜色数据
 * @param  red   红色分量 (0-255)
 * @param  green 绿色分量 (0-255)
 * @param  blue  蓝色分量 (0-255)
 *
 * @details 
 */
void LightWs2812(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t buffer[124] = {0}; /*!< 后 100 字节全 0，作为 WS2812 复位信号 */
    for (int i = 0; i < 8; i++)
    {
        /* 按 GRB 顺序编码：Green -> Red -> Blue，每个颜色 MSB 优先 */
        buffer[7-i]  = (((green>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        buffer[15-i] = (((red>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        buffer[23-i] = (((blue>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
    }
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);  /*!< 等待 SPI 总线空闲 */
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, buffer, 124, 0xFFFF); /*!< 阻塞发送，超时 ~65535ms */
}

/**
 * @brief  BDMA 内存段中的 WS2812 发送缓冲区
 *
 * @details STM32H723 的 BDMA 只能访问 SRAM4 区域。
 *          使用 section 属性指定内存位置，aligned(32) 确保 DMA 对齐优化。
 *          该缓冲区为全局静态变量，可被 DMA 和 CPU 同时访问，
 *          调用者需通过检查 SPI 状态确保不会与正在进行的 DMA 传输冲突。
 */
__attribute__((section(".bdma_buf"), aligned(32))) uint8_t bdma_buffer[124];

/**
 * @brief  DMA 模式发送 1 颗 WS2812 LED 颜色数据
 * @param  red   红色分量 (0-255)
 * @param  green 绿色分量 (0-255)
 * @param  blue  蓝色分量 (0-255)
 *
 * @details 
 */
void LightWs2812BDMA(uint8_t red, uint8_t green, uint8_t blue)
{
    memset(bdma_buffer, 0, 124); /*!< 清零全部缓冲区，后 100 字节为复位信号 */
    for (int i = 0; i < 8; i++)
    {
        /* GRB 顺序，MSB 优先编码 */
        bdma_buffer[7-i]  = (((green>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        bdma_buffer[15-i] = (((red>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        bdma_buffer[23-i] = (((blue>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
    }
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);  /*!< 等待前一次传输完成 */
    HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, bdma_buffer, 124);

    return;
}
