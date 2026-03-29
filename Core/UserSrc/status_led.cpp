#include "status_led.hpp"
#include "ws2812.h"

StatusLed::StatusLed()
{
    status_idx_ = 255;
}


void StatusLed::UpdateColorBDMA(uint8_t status_idx)
{
    if ((status_idx < 9) && (status_idx != status_idx_))
    {
        LightWs2812BDMA(kRGBColors[status_idx][0], kRGBColors[status_idx][1], kRGBColors[status_idx][2]);
    }
}

void StatusLed::UpdateColor(uint8_t status_idx)
{
    if ((status_idx < 9) && (status_idx != status_idx_))
    {
        LightWs2812BDMA(kRGBColors[status_idx][0], kRGBColors[status_idx][1], kRGBColors[status_idx][2]);
    }
}
