#ifndef PCA9548A_HPP
#define PCA9548A_HPP

#include "main.h"

class PCA9548A {
public:
    PCA9548A();
    ~PCA9548A();

    bool begin(I2C_HandleTypeDef* hi2c, uint8_t addr_offset = 0, GPIO_TypeDef* rst_port = nullptr, uint16_t rst_pin = 255);

    bool reset();
    bool selectChannel(uint8_t ch);
    bool selectMultiChannels(uint8_t mask);

private:
    I2C_HandleTypeDef* _hi2c = nullptr;
    GPIO_TypeDef* _rst_port  = nullptr;
    uint16_t _rst_pin = 255;
    
    uint8_t _addr_offset = 0;
    uint8_t _addr = 0;

    static constexpr uint8_t  kBaseAddr = 0x70;
    static constexpr uint32_t kTimeoutMs = 10; 
    static constexpr uint32_t kResetTimeLowMs = 1;
};

#endif