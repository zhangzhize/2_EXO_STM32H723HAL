#ifndef WS2812_H
#define WS2812_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

void LightWs2812(uint8_t red, uint8_t green, uint8_t blue);
void LightWs2812BDMA(uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif

#endif
