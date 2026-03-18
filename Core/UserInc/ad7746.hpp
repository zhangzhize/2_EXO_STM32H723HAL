#ifndef AD7746_HPP
#define AD7746_HPP

#include "main.h"

enum class AD7746_ERROR : int8_t {
    OK           = 0,
    INIT_ERROR   = -1,
    BUS_ERROR    = -2,
    TIMEOUT      = -3
};

class AD7746 {
public:
    AD7746();
    ~AD7746() = default;

    AD7746_ERROR begin(I2C_HandleTypeDef* hi2c);
    AD7746_ERROR reset();

    AD7746_ERROR writeRegister(uint8_t reg, uint8_t value);
    AD7746_ERROR readRegister(uint8_t reg, uint8_t* value);

    AD7746_ERROR readCapacitanceRaw(uint32_t& raw_data);
    AD7746_ERROR readCapacitance(float& cap_value, uint8_t capdac = 0x10);
    AD7746_ERROR readTemperatureRaw(uint32_t& raw_data);

private:
    I2C_HandleTypeDef* _hi2c = nullptr;
    static constexpr uint8_t _addr = 0x48;

    static constexpr uint8_t CMD_RESET         = 0xBF;

    static constexpr uint8_t REG_STATUS        = 0x00;
    static constexpr uint8_t REG_CAP_DATA_H    = 0x01;
    static constexpr uint8_t REG_CAP_DATA_M    = 0x02;
    static constexpr uint8_t REG_CAP_DATA_L    = 0x03;
    static constexpr uint8_t REG_VT_DATA_H     = 0x04;
    static constexpr uint8_t REG_VT_DATA_M     = 0x05;
    static constexpr uint8_t REG_VT_DATA_L     = 0x06;
    static constexpr uint8_t REG_CAP_SETUP     = 0x07;
    static constexpr uint8_t REG_VT_SETUP      = 0x08;
    static constexpr uint8_t REG_EXC_SETUP     = 0x09;
    static constexpr uint8_t REG_CONFIGURATION = 0x0A;
    static constexpr uint8_t REG_CAPDAC_A      = 0x0B;
    static constexpr uint8_t REG_CAPDAC_B      = 0x0C;
};

#endif