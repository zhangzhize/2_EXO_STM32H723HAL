#include "ws2812.h"
#include "spi.h"
#include <string.h>

#define WS2812_LOW_LEVEL    (0xC0)     // 0码
#define WS2812_HIGH_LEVEL   (0xF0)     // 1码

#define WS2812_SPI_UNIT   hspi6

void LightWs2812(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t buffer[124] = {0}; // 后100字节全0，为复位信号
    for (int i = 0; i < 8; i++)
    {
        buffer[7-i]  = (((green>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        buffer[15-i] = (((red>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        buffer[23-i] = (((blue>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
    }
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, buffer, 124, 0xFFFF);
}

__attribute__((section(".bdma_buf"), aligned(32))) uint8_t bdma_buffer[124];

void LightWs2812BDMA(uint8_t red, uint8_t green, uint8_t blue)
{
    memset(bdma_buffer, 0, 124); // 后100字节全0，为复位信号
    for (int i = 0; i < 8; i++)
    {
        bdma_buffer[7-i]  = (((green>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        bdma_buffer[15-i] = (((red>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        bdma_buffer[23-i] = (((blue>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
    }
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, bdma_buffer, 124);

    return;
}