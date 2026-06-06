#include "status_led.hpp"
#include "ws2812.h"

/**
 * @brief DMA+PWM 方式更新 LED (后台传输，不占 CPU)
 * @param status_idx  状态索引 (0~8，对应 colors 查找表)
 */
void StateLed::UpdateColorBDMA(uint8_t status_idx)
{
    /* 仅当状态确实变化且索引有效时才触发硬件更新 */
    if ((status_idx < kColorCount) && (status_idx != status_idx_))
    {
        LightWs2812BDMA(kRGBColors[status_idx][0], kRGBColors[status_idx][1], kRGBColors[status_idx][2]);
        status_idx_ = status_idx;
    }
}

void StateLed::UpdateEmergencyStopColorBDMA()
{
    UpdateColorBDMA(kEmergencyStopColorIdx);
}

/**
 * @brief Bit-bang 方式更新 LED (CPU 直接操作 GPIO)
 * @param status_idx  状态索引 (0~8，对应 colors 查找表)
 */
void StateLed::UpdateColor(uint8_t status_idx)
{
    /* 仅当状态确实变化且索引有效时才触发硬件更新 */
    if ((status_idx < kColorCount) && (status_idx != status_idx_))
    {
        LightWs2812(kRGBColors[status_idx][0], kRGBColors[status_idx][1], kRGBColors[status_idx][2]);
        status_idx_ = status_idx;
    }
}

void StateLed::UpdateEmergencyStopColor()
{
    UpdateColor(kEmergencyStopColorIdx);
}
