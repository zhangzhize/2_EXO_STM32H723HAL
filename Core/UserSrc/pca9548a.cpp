#include "pca9548a.hpp"

PCA9548A::PCA9548A() {}

PCA9548A::~PCA9548A() {
    if (_rst_port != nullptr && _rst_pin != 255) {
        HAL_GPIO_WritePin(_rst_port, _rst_pin, GPIO_PIN_RESET);
    }
}

bool PCA9548A::begin(I2C_HandleTypeDef* hi2c, uint8_t addr_offset, GPIO_TypeDef* rst_port, uint16_t rst_pin) {
    if (hi2c == nullptr) {
        return false;
    }
    if (rst_port != nullptr && rst_pin != 255) {
        _rst_port = rst_port;
        _rst_pin = rst_pin;
    }
    _hi2c = hi2c;
    _addr_offset = addr_offset;
    _addr = (BASE_ADDR + _addr_offset) << 1;

    return reset();
}

bool PCA9548A::reset() {
    if (_rst_port != nullptr && _rst_pin != 255) {
        HAL_GPIO_WritePin(_rst_port, _rst_pin, GPIO_PIN_RESET);
        HAL_Delay(RESET_TIME_LOW_MS);
        HAL_GPIO_WritePin(_rst_port, _rst_pin, GPIO_PIN_SET);
    }

    return selectMultiChannels(0);
}

bool PCA9548A::selectChannel(uint8_t ch) {
    uint8_t mask = 0;
    if (ch <= 7) {
        mask = 1 << ch;
    }
    
    return selectMultiChannels(mask);
}

bool PCA9548A::selectMultiChannels(uint8_t mask) {
    if (_hi2c == nullptr) {
        return false;
    }

    if (HAL_I2C_Master_Transmit(_hi2c, _addr, &mask, 1, TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    uint8_t mask_check = 0;
    if (HAL_I2C_Master_Receive(_hi2c, _addr, &mask_check, 1, TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    if (mask_check != mask) {
        return false;
    }

    return true;
}