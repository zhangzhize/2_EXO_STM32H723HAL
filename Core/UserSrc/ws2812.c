#include "ws2812.h"
#include "spi.h"
#include <string.h>

#define WS2812_LOW_LEVEL    (0xC0)     // 0码
#define WS2812_HIGH_LEVEL   (0xF0)     // 1码

#define WS2812_SPI_UNIT   hspi6

void LightWs2812(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t ucaTxBuf[24];
    uint8_t uRes = 0;
    for (int i = 0; i < 8; i++)
    {
        ucaTxBuf[7-i]  = (((green>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        ucaTxBuf[15-i] = (((red>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        ucaTxBuf[23-i] = (((blue>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
    }
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, ucaTxBuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &uRes, 1, 0xFFFF);
    }
}

__attribute__((section(".sram4"), aligned(32)))uint8_t ucaTxBuf[24];

__attribute__((section(".sram4"), aligned(32)))uint8_t ucaResBuf[100] = {0};

void LightWs2812BDMA(uint8_t red, uint8_t green, uint8_t blue)
{
    for (int i = 0; i < 8; i++)
    {
        ucaTxBuf[7-i]  = (((green>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        ucaTxBuf[15-i] = (((red>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
        ucaTxBuf[23-i] = (((blue>>i)&0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL)>>1;
    }
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, ucaTxBuf, 24);

    memset(ucaResBuf, 0, 100);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, ucaResBuf, 100);
}