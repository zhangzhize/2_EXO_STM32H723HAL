/**
 * @file    status_led.hpp
 * @brief   WS2812B 状态指示灯控制模块
 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <cstdint>

/**
 * @class StateLed
 * @brief 状态指示灯驱动（WS2812B 单灯）
 *
 * 使用预定义的 9 色查找表，根据系统状态索引更新 LED 颜色。
 */
class StateLed
{
public:
    explicit StateLed() {}    /* 初始化为无效值，确保首次调用触发更新 */
    virtual ~StateLed() = default;

    void UpdateColor(uint8_t status_idx);
    void UpdateColorBDMA(uint8_t status_idx);
    void UpdateEmergencyStopColor();
    void UpdateEmergencyStopColorBDMA();
private:
    static constexpr uint8_t kColorCount = 9;
    static constexpr uint8_t kEmergencyStopColorIdx = kColorCount - 1;

    uint8_t status_idx_ = 255;                /*!< 上一次更新的状态索引，用于变化检测 */

    /* RGB 查找表: 普通状态颜色放前面，最后一项固定保留给急停 */
    const uint8_t kRGBColors[kColorCount][3] = {
        {0x00, 0x00, 0x00}, /* 熄灭       #000000 */
        {0x00, 0xFF, 0x00}, /* 绿色       #00FF00 */
        {0x00, 0x00, 0xFF}, /* 蓝色       #0000FF */
        {0xFF, 0xFF, 0x00}, /* 黄色       #FFFF00 */
        {0xFF, 0x00, 0xFF}, /* 品红色     #FF00FF */
        {0x00, 0xFF, 0xFF}, /* 青色       #00FFFF */
        {0xFF, 0x8C, 0x00}, /* 橙色       #FF8C00 */
        {0x80, 0x00, 0x80}, /* 保留紫色   #800080 */
        {0xFF, 0x00, 0x00}, /* 急停红色   #FF0000 */
    };
};


#endif
