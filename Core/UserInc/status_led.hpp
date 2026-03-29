#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <cstdint>

class StatusLed
{
public:
    StatusLed();
    ~StatusLed() = default;

    void UpdateColor(uint8_t status_idx);
    void UpdateColorBDMA(uint8_t status_idx);
    
    uint8_t status_idx_;
    
    const uint8_t kRGBColors[9][3] = {
    {  0,   0,   0}, // Off       #000000
    {  0, 100,   0}, // Green     #00FF00
    {  0,   0, 100}, // Blue      #0000FF
    {100, 100,   0}, // Yellow    #FFFF00
    {100,   0, 100}, // Magenta   #FF00FF
    {  0, 100, 100}, // Cyan      #00FFFF
    {255, 140,   0}, // Orange    #FF8C00
    {128,   0, 128}, // Purple    #800080
    {255,   0,   0}, // Red       #FF0000
};
};


#endif
