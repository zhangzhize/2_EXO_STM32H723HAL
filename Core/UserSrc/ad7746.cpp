#include "ad7746.hpp"

AD7746::AD7746() {}

AD7746_ERROR AD7746::begin(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == nullptr)
        return AD7746_ERROR::INIT_ERROR;

    _hi2c = hi2c;

    if (reset() != AD7746_ERROR::NO_ERROR)
    {
        return AD7746_ERROR::BUS_ERROR;
    }

    HAL_Delay(5);
    AD7746_ERROR err = AD7746_ERROR::NO_ERROR;
    err = writeRegister(REG_CAP_SETUP, 0x80); // CAPEN_ON
    if (err != AD7746_ERROR::NO_ERROR)
        return err;
    err = writeRegister(REG_EXC_SETUP, 0x0F); // EXCON_ON | nEXCB_ON | EXCA_ON | EXCLVL_VDD_2
    if (err != AD7746_ERROR::NO_ERROR)
        return err;
    err = writeRegister(REG_CONFIGURATION, 0x21); // CONF_CAPF_62_MS (0x20) | MD_CONTINUOUS_CONVERSION (0x01)
    if (err != AD7746_ERROR::NO_ERROR)
        return err;
    err = writeRegister(REG_CAPDAC_A, 0x90); // DACAENA_ON (0x80) | 0x10 = 0x90
    if (err != AD7746_ERROR::NO_ERROR)
        return err;

    return AD7746_ERROR::NO_ERROR;
}

AD7746_ERROR AD7746::reset()
{
    if (_hi2c == nullptr)
        return AD7746_ERROR::INIT_ERROR;

    uint8_t cmd = CMD_RESET;
    if (HAL_I2C_Master_Transmit(_hi2c, _addr << 1, &cmd, 1, 100) == HAL_OK)
    {
        HAL_Delay(2);
        return AD7746_ERROR::NO_ERROR;
    }
    return AD7746_ERROR::BUS_ERROR;
}

AD7746_ERROR AD7746::writeRegister(uint8_t reg, uint8_t value)
{
    if (_hi2c == nullptr)
        return AD7746_ERROR::INIT_ERROR;

    if (HAL_I2C_Mem_Write(_hi2c, _addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) == HAL_OK)
    {
        return AD7746_ERROR::NO_ERROR;
    }
    return AD7746_ERROR::BUS_ERROR;
}

AD7746_ERROR AD7746::readRegister(uint8_t reg, uint8_t *value)
{
    if (_hi2c == nullptr)
        return AD7746_ERROR::INIT_ERROR;

    if (HAL_I2C_Mem_Read(_hi2c, _addr << 1, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100) == HAL_OK)
    {
        return AD7746_ERROR::NO_ERROR;
    }
    return AD7746_ERROR::BUS_ERROR;
}

AD7746_ERROR AD7746::readCapacitanceRaw(uint32_t &raw_data)
{
    if (_hi2c == nullptr)
        return AD7746_ERROR::INIT_ERROR;

    uint8_t buffer[3] = {0};

    if (HAL_I2C_Mem_Read(_hi2c, _addr << 1, REG_CAP_DATA_H, I2C_MEMADD_SIZE_8BIT, buffer, 3, 100) != HAL_OK)
    {
        return AD7746_ERROR::BUS_ERROR;
    }

    raw_data = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    return AD7746_ERROR::NO_ERROR;
}

AD7746_ERROR AD7746::readCapacitance(float &cap_value, uint8_t capdac)
{
    uint32_t raw_data = 0;
    AD7746_ERROR err = readCapacitanceRaw(raw_data);
    if (err != AD7746_ERROR::NO_ERROR)
        return err;

    cap_value = ((float)raw_data / 16777216.0f) * 8.0f + ((float)capdac - 4.0f);

    return AD7746_ERROR::NO_ERROR;
}

AD7746_ERROR AD7746::readTemperatureRaw(uint32_t &raw_data)
{
    if (_hi2c == nullptr)
        return AD7746_ERROR::INIT_ERROR;

    uint8_t buffer[3] = {0};

    if (HAL_I2C_Mem_Read(_hi2c, _addr << 1, REG_VT_DATA_H, I2C_MEMADD_SIZE_8BIT, buffer, 3, 100) != HAL_OK)
    {
        return AD7746_ERROR::BUS_ERROR;
    }

    raw_data = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    return AD7746_ERROR::NO_ERROR;
}