#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <cstdint>

class StateLed
{
public:
    explicit StateLed() : status_idx_(255) {}
    virtual ~StateLed() = default;

    void UpdateColor(uint8_t status_idx);
    void UpdateColorBDMA(uint8_t status_idx);
private:
    uint8_t status_idx_;

    const uint8_t kRGBColors[9][3] = {
        {0x00, 0x00, 0x00}, // Off       #000000
        {0x00, 0xFF, 0x00}, // Green     #00FF00
        {0x00, 0x00, 0xFF}, // Blue      #0000FF
        {0xFF, 0xFF, 0x00}, // Yellow    #FFFF00
        {0xFF, 0x00, 0xFF}, // Magenta   #FF00FF
        {0x00, 0xFF, 0xFF}, // Cyan      #00FFFF
        {0xFF, 0x8C, 0x00}, // Orange    #FF8C00
        {0x80, 0x00, 0x80}, // Purple    #800080
        {0xFF, 0x00, 0x00}, // Red       #FF0000
    };
};


#endif
