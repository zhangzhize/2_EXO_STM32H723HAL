#include "as7341.hpp"
#include "utils.h"

AS7341::AS7341()
{
}

AS7341_ERROR AS7341::begin(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    if (hi2c == nullptr)
    {
        return AS7341_ERROR::I2C_COMM_ERROR;
    }

    _hi2c = hi2c;
    _addr = addr;

    if (!isConnected())
    {
        return AS7341_ERROR::WRONG_CHIP_ID;
    }

    enable_AS7341();
    // Set default ADC integration steps registers (599)
    setASTEP();
    // Set default ADC integration time register (29)
    setATIME();
    // Set ADC gain to default (x256)
    setGain();

    // Have AS7341 control external led.
    // setRegisterBit(REGISTER_CONFIG, 3);

    return AS7341_ERROR::OK;
}

bool AS7341::isConnected()
{
    uint8_t chip_id = 0;
    chip_id = readRegister(REGISTER_ID);
    return ((chip_id >> 2) == 0x09);
}

void AS7341::enable_AS7341()
{
    setRegisterBit(REGISTER_ENABLE, 0);
}

void AS7341::disable_AS7341()
{
    clearRegisterBit(REGISTER_ENABLE, 0);
}

void AS7341::setLedDrive(unsigned int current)
{
    // Do not allow invalid values to be set
    if (current < 4)
        current = 4;
    if (current > 258)
        current = 258;

    // Calculate register value to program
    uint8_t registerValue = uint8_t((current - 4) >> 1);
    // Read current REGISTER_LED value
    uint8_t currentValue = readSingleByte(REGISTER_LED);
    // Clear bits 6:0
    currentValue &= 0x80;
    // Update bits 6:0 accordingly
    currentValue |= registerValue;
    // Write register back
    writeSingleByte(REGISTER_LED, registerValue);
}

unsigned int AS7341::getLedDrive()
{
    uint8_t currentRegister = readSingleByte(REGISTER_LED);
    currentRegister &= 0x7f;
    return (currentRegister << 1) + 4;
}

void AS7341::setATIME(uint8_t aTime /* = 29 */)
{
    writeSingleByte(REGISTER_ATIME, aTime);
}

void AS7341::setASTEP(unsigned int aStep /* = 599 */)
{
    uint8_t temp = uint8_t(aStep >> 8);
    writeSingleByte(REGISTER_ASTEP_H, temp);
    temp = uint8_t(aStep &= 0xff);
    writeSingleByte(REGISTER_ASTEP_L, temp);
}

uint8_t AS7341::getATIME()
{
    return readSingleByte(REGISTER_ATIME);
}

unsigned int AS7341::getASTEP()
{
    unsigned int result = readSingleByte(REGISTER_ASTEP_H);
    result = result << 8;
    result |= readSingleByte(REGISTER_ASTEP_L);
    return result;
}

void AS7341::setGain(AS7341_GAIN gain)
{
    uint8_t value;

    switch (gain)
    {
    case AS7341_GAIN::GAIN_HALF:
        value = 0;
        break;

    case AS7341_GAIN::GAIN_X1:
        value = 1;
        break;

    case AS7341_GAIN::GAIN_X2:
        value = 2;
        break;

    case AS7341_GAIN::GAIN_X4:
        value = 3;
        break;

    case AS7341_GAIN::GAIN_X8:
        value = 4;
        break;

    case AS7341_GAIN::GAIN_X16:
        value = 5;
        break;

    case AS7341_GAIN::GAIN_X32:
        value = 6;
        break;

    case AS7341_GAIN::GAIN_X64:
        value = 7;
        break;

    case AS7341_GAIN::GAIN_X128:
        value = 8;
        break;

    case AS7341_GAIN::GAIN_X256:
        value = 9;
        break;

    case AS7341_GAIN::GAIN_X512:
        value = 10;
        break;

    case AS7341_GAIN::GAIN_INVALID:
    default:
        return;
    }

    writeSingleByte(REGISTER_CFG_1, value);
}

AS7341_GAIN AS7341::getGain()
{
    uint8_t value = readSingleByte(REGISTER_CFG_1);
    value &= 0x1f;
    AS7341_GAIN returnValue;

    switch (value)
    {
    case 0:
        returnValue = AS7341_GAIN::GAIN_HALF;
        break;

    case 1:
        returnValue = AS7341_GAIN::GAIN_X1;
        break;

    case 2:
        returnValue = AS7341_GAIN::GAIN_X2;
        break;

    case 3:
        returnValue = AS7341_GAIN::GAIN_X4;
        break;

    case 4:
        returnValue = AS7341_GAIN::GAIN_X8;
        break;

    case 5:
        returnValue = AS7341_GAIN::GAIN_X16;
        break;

    case 6:
        returnValue = AS7341_GAIN::GAIN_X32;
        break;

    case 7:
        returnValue = AS7341_GAIN::GAIN_X64;
        break;

    case 8:
        returnValue = AS7341_GAIN::GAIN_X128;
        break;

    case 9:
        returnValue = AS7341_GAIN::GAIN_X256;
        break;

    case 10:
        returnValue = AS7341_GAIN::GAIN_X512;
        break;

    default:
        returnValue = AS7341_GAIN::GAIN_INVALID;
        break;
    }

    return returnValue;
}

AS7341_ERROR AS7341::readAllChannels(unsigned int *channelData)
{
    setMuxLo();
    DelayMs(5);
    setRegisterBit(REGISTER_ENABLE, 1);
    unsigned int start = GetSysTimeMs();
    unsigned int delta = 0;
    do
    {
        delta = GetSysTimeMs();
        if (delta - start > 5000)
        {
            return AS7341_ERROR::MEASUREMENT_TIMEOUT;
        }
    } while (!isBitSet(REGISTER_STATUS_2, 6));

    uint8_t buffer[12];
    readMultipleBytes(REGISTER_CH0_DATA_L, buffer, 12);
    for (int i = 0; i < 6; i++)
        channelData[i] = buffer[2 * i + 1] << 8 | buffer[2 * i];

    setMuxHi();
    DelayMs(5);
    setRegisterBit(REGISTER_ENABLE, 1);
    start = GetSysTimeMs();
    delta = 0;
    do
    {
        delta = GetSysTimeMs();
        if (delta - start > 5000)
        {
            return AS7341_ERROR::MEASUREMENT_TIMEOUT;
        }
    } while (!isBitSet(REGISTER_STATUS_2, 6));
    readMultipleBytes(REGISTER_CH0_DATA_L, buffer, 12);
    for (int i = 0; i < 6; i++)
        channelData[i + 6] = buffer[2 * i + 1] << 8 | buffer[2 * i];

    return AS7341_ERROR::OK;
}

void AS7341::setMuxLo()
{
    // According to AMS application note V1.1
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x30);
    writeSingleByte(0x01, 0x01);
    writeSingleByte(0x02, 0x00);
    writeSingleByte(0x03, 0x00);
    writeSingleByte(0x04, 0x00);
    writeSingleByte(0x05, 0x42);
    writeSingleByte(0x06, 0x00);
    writeSingleByte(0x07, 0x00);
    writeSingleByte(0x08, 0x50);
    writeSingleByte(0x09, 0x00);
    writeSingleByte(0x0a, 0x00);
    writeSingleByte(0x0b, 0x00);
    writeSingleByte(0x0c, 0x20);
    writeSingleByte(0x0d, 0x04);
    writeSingleByte(0x0e, 0x00);
    writeSingleByte(0x0f, 0x30);
    writeSingleByte(0x10, 0x01);
    writeSingleByte(0x11, 0x50);
    writeSingleByte(0x12, 0x00);
    writeSingleByte(0x13, 0x06);
    writeSingleByte(REGISTER_ENABLE, 0x11);
}

void AS7341::setMuxHi()
{
    // According to AMS application note V1.1
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x00, 0x00);
    writeSingleByte(0x01, 0x00);
    writeSingleByte(0x02, 0x00);
    writeSingleByte(0x03, 0x40);
    writeSingleByte(0x04, 0x02);
    writeSingleByte(0x05, 0x00);
    writeSingleByte(0x06, 0x10);
    writeSingleByte(0x07, 0x03);
    writeSingleByte(0x08, 0x50);
    writeSingleByte(0x09, 0x10);
    writeSingleByte(0x0a, 0x03);
    writeSingleByte(0x0b, 0x00);
    writeSingleByte(0x0c, 0x00);
    writeSingleByte(0x0d, 0x00);
    writeSingleByte(0x0e, 0x24);
    writeSingleByte(0x0f, 0x00);
    writeSingleByte(0x10, 0x00);
    writeSingleByte(0x11, 0x50);
    writeSingleByte(0x12, 0x00);
    writeSingleByte(0x13, 0x06);
    writeSingleByte(REGISTER_ENABLE, 0x11);
}

AS7341_ERROR AS7341::readAllChannelsBasicCounts(float *channelDataBasicCounts)
{

    unsigned int rawChannelData[12];
    AS7341_ERROR result = readAllChannels(rawChannelData);

    if (result != AS7341_ERROR::OK)
        return result;

    unsigned int aTime = (unsigned int)getATIME();
    unsigned int aStep = getASTEP();
    unsigned int tint = (aTime + 1) * (aStep * 1) * 2.78 / 1000;

    uint8_t value = readSingleByte(REGISTER_CFG_1);
    value &= 0x1f;

    float gain;
    if (value == 0)
    {
        gain = 0.5f;
    }
    else
    {
        gain = pow(2, (value - 1));
    }

    for (int i = 0; i < 12; i++)
    {
        channelDataBasicCounts[i] = float(rawChannelData[i]) / (gain * tint);
    }

    return AS7341_ERROR::OK;
}

void AS7341::enablePinInterupt()
{
    setRegisterBit(REGISTER_INTENAB, 0);
}

void AS7341::disablePinInterrupt()
{
    clearRegisterBit(REGISTER_INTENAB, 0);
}

void AS7341::clearPinInterrupt()
{
    writeSingleByte(REGISTER_STATUS, 0xff);
}

uint8_t AS7341::readRegister(uint8_t reg)
{
    return readSingleByte(reg);
}

void AS7341::writeRegister(uint8_t reg, uint8_t value)
{
    writeSingleByte(reg, value);
}

void AS7341::setGpioPinInput()
{
    // Disable GPIO as output driver
    setRegisterBit(REGISTER_GPIO_2, 1);
    // Enable GPIO as input
    setRegisterBit(REGISTER_GPIO_2, 2);
}

void AS7341::setGpioPinOutput()
{
    // Disable GPIO input
    clearRegisterBit(REGISTER_GPIO_2, 2);
}

bool AS7341::digitalRead()
{
    return isBitSet(REGISTER_GPIO_2, 0);
}

void AS7341::invertGpioOutput(bool isInverted)
{
    if (isInverted)
        setRegisterBit(REGISTER_GPIO_2, 3);
    else
        clearRegisterBit(REGISTER_GPIO_2, 3);
}

void AS7341::digitalWrite(uint8_t value)
{
    // If value is 0, let go the driver
    if (value == 0)
        clearRegisterBit(REGISTER_GPIO_2, 1);
    else
        setRegisterBit(REGISTER_GPIO_2, 1);
}

uint16_t AS7341::readSingleChannelValue()
{
    setRegisterBit(REGISTER_ENABLE, 1);

    uint16_t start = GetSysTimeMs();
    uint16_t delta = 0;

    do
    {
        delta = GetSysTimeMs();
        if (delta - start > 5000)
        {
            return 0;
        }
    } while (!isBitSet(REGISTER_STATUS_2, 6));

    uint8_t buffer[2] = {0};
    readMultipleBytes(REGISTER_CH0_DATA_L, buffer, 2);
    uint16_t result = buffer[1] << 8;
    result += buffer[0];

    return result;
}

unsigned int AS7341::read415nm()
{
    // F1 -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x00);
    writeSingleByte(0x01, 0x01);
    writeSingleByte(0x02, 0x00);
    writeSingleByte(0x03, 0x00);
    writeSingleByte(0x04, 0x00);
    writeSingleByte(0x05, 0x00);
    writeSingleByte(0x06, 0x00);
    writeSingleByte(0x07, 0x00);
    writeSingleByte(0x08, 0x00);
    writeSingleByte(0x09, 0x00);
    writeSingleByte(0x0a, 0x00);
    writeSingleByte(0x0b, 0x00);
    writeSingleByte(0x0c, 0x00);
    writeSingleByte(0x0d, 0x00);
    writeSingleByte(0x0e, 0x00);
    writeSingleByte(0x0f, 0x00);
    writeSingleByte(0x10, 0x01);
    writeSingleByte(0x11, 0x00);
    writeSingleByte(0x12, 0x00);
    writeSingleByte(0x13, 0x00);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::read445nm()
{
    // F2 -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x0);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x0);
    writeSingleByte(0x04, 0x0);
    writeSingleByte(0x05, 0x01);
    writeSingleByte(0x06, 0x0);
    writeSingleByte(0x07, 0x0);
    writeSingleByte(0x08, 0x0);
    writeSingleByte(0x09, 0x0);
    writeSingleByte(0x0a, 0x0);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x10);
    writeSingleByte(0x0d, 0x0);
    writeSingleByte(0x0e, 0x0);
    writeSingleByte(0x0f, 0x0);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x0);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x0);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::read480nm()
{
    // F3 -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x10);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x0);
    writeSingleByte(0x04, 0x0);
    writeSingleByte(0x05, 0x0);
    writeSingleByte(0x06, 0x0);
    writeSingleByte(0x07, 0x0);
    writeSingleByte(0x08, 0x0);
    writeSingleByte(0x09, 0x0);
    writeSingleByte(0x0a, 0x0);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x0);
    writeSingleByte(0x0d, 0x0);
    writeSingleByte(0x0e, 0x0);
    writeSingleByte(0x0f, 0x10);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x0);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x0);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::read515nm()
{
    // F4 -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x0);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x0);
    writeSingleByte(0x04, 0x0);
    writeSingleByte(0x05, 0x10);
    writeSingleByte(0x06, 0x0);
    writeSingleByte(0x07, 0x0);
    writeSingleByte(0x08, 0x0);
    writeSingleByte(0x09, 0x0);
    writeSingleByte(0x0a, 0x0);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x0);
    writeSingleByte(0x0d, 0x01);
    writeSingleByte(0x0e, 0x0);
    writeSingleByte(0x0f, 0x0);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x0);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x0);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::read555nm()
{
    // F5 -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x0);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x0);
    writeSingleByte(0x04, 0x0);
    writeSingleByte(0x05, 0x0);
    writeSingleByte(0x06, 0x10);
    writeSingleByte(0x07, 0x0);
    writeSingleByte(0x08, 0x0);
    writeSingleByte(0x09, 0x10);
    writeSingleByte(0x0a, 0x0);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x0);
    writeSingleByte(0x0d, 0x0);
    writeSingleByte(0x0e, 0x0);
    writeSingleByte(0x0f, 0x0);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x0);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x0);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::read590nm()
{
    // F6 -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x0);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x0);
    writeSingleByte(0x04, 0x01);
    writeSingleByte(0x05, 0x0);
    writeSingleByte(0x06, 0x0);
    writeSingleByte(0x07, 0x0);
    writeSingleByte(0x08, 0x0);
    writeSingleByte(0x09, 0x0);
    writeSingleByte(0x0a, 0x0);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x0);
    writeSingleByte(0x0d, 0x0);
    writeSingleByte(0x0e, 0x10);
    writeSingleByte(0x0f, 0x0);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x0);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x0);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::read630nm()
{
    // F7 -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x0);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x0);
    writeSingleByte(0x04, 0x0);
    writeSingleByte(0x05, 0x0);
    writeSingleByte(0x06, 0x0);
    writeSingleByte(0x07, 0x01);
    writeSingleByte(0x08, 0x0);
    writeSingleByte(0x09, 0x0);
    writeSingleByte(0x0a, 0x01);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x0);
    writeSingleByte(0x0d, 0x0);
    writeSingleByte(0x0e, 0x0);
    writeSingleByte(0x0f, 0x0);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x0);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x0);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::read680nm()
{
    // F8 -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x0);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x10);
    writeSingleByte(0x04, 0x0);
    writeSingleByte(0x05, 0x0);
    writeSingleByte(0x06, 0x0);
    writeSingleByte(0x07, 0x0);
    writeSingleByte(0x08, 0x0);
    writeSingleByte(0x09, 0x0);
    writeSingleByte(0x0a, 0x0);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x0);
    writeSingleByte(0x0d, 0x0);
    writeSingleByte(0x0e, 0x01);
    writeSingleByte(0x0f, 0x0);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x0);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x0);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::readClear()
{
    //	Clear -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x0);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x0);
    writeSingleByte(0x04, 0x0);
    writeSingleByte(0x05, 0x0);
    writeSingleByte(0x06, 0x0);
    writeSingleByte(0x07, 0x0);
    writeSingleByte(0x08, 0x10);
    writeSingleByte(0x09, 0x0);
    writeSingleByte(0x0a, 0x0);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x0);
    writeSingleByte(0x0d, 0x0);
    writeSingleByte(0x0e, 0x0);
    writeSingleByte(0x0f, 0x0);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x10);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x0);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

unsigned int AS7341::readNIR()
{
    //	NIR -> ADC0
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x0, 0x0);
    writeSingleByte(0x01, 0x0);
    writeSingleByte(0x02, 0x0);
    writeSingleByte(0x03, 0x0);
    writeSingleByte(0x04, 0x0);
    writeSingleByte(0x05, 0x0);
    writeSingleByte(0x06, 0x0);
    writeSingleByte(0x07, 0x0);
    writeSingleByte(0x08, 0x0);
    writeSingleByte(0x09, 0x0);
    writeSingleByte(0x0a, 0x0);
    writeSingleByte(0x0b, 0x0);
    writeSingleByte(0x0c, 0x0);
    writeSingleByte(0x0d, 0x0);
    writeSingleByte(0x0e, 0x0);
    writeSingleByte(0x0f, 0x0);
    writeSingleByte(0x10, 0x0);
    writeSingleByte(0x11, 0x0);
    writeSingleByte(0x12, 0x0);
    writeSingleByte(0x13, 0x01);
    writeSingleByte(REGISTER_ENABLE, 0x11);

    return (unsigned int)readSingleChannelValue();
}

float AS7341::readSingleBasicCountChannelValue(uint16_t raw)
{
    uint16_t aTime = (uint16_t)getATIME();
    uint16_t aStep = getASTEP();
    uint16_t tint = (aTime + 1) * (aStep * 1) * 2.78 / 1000;

    uint8_t value = readSingleByte(REGISTER_CFG_1);
    value &= 0x1f;

    float gain;
    if (value == 0)
    {
        gain = 0.5f;
    }
    else
    {
        gain = pow(2, (value - 1));
    }

    return (float(raw) / (gain * tint));
}

float AS7341::readBasicCount415nm()
{
    return readSingleBasicCountChannelValue(read415nm());
}

float AS7341::readBasicCount445nm()
{
    return readSingleBasicCountChannelValue(read445nm());
}

float AS7341::readBasicCount480nm()
{
    return readSingleBasicCountChannelValue(read480nm());
}

float AS7341::readBasicCount515nm()
{
    return readSingleBasicCountChannelValue(read515nm());
}

float AS7341::readBasicCount555nm()
{
    return readSingleBasicCountChannelValue(read555nm());
}

float AS7341::readBasicCount590nm()
{
    return readSingleBasicCountChannelValue(read590nm());
}

float AS7341::readBasicCount630nm()
{
    return readSingleBasicCountChannelValue(read630nm());
}

float AS7341::readBasicCount680nm()
{
    return readSingleBasicCountChannelValue(read680nm());
}

float AS7341::readBasicCountClear()
{
    return readSingleBasicCountChannelValue(readClear());
}

float AS7341::readBasicCountNIR()
{
    return readSingleBasicCountChannelValue(readNIR());
}

void AS7341::setLowThreshold(unsigned int threshold)
{
    uint8_t low = threshold & 0xff;
    uint8_t high = threshold >> 8;

    writeSingleByte(REGISTER_SP_TH_L_LSB, low);
    writeSingleByte(REGISTER_SP_TH_L_MSB, high);
}

void AS7341::setHighThreshold(unsigned int threshold)
{
    uint8_t low = threshold & 0xff;
    uint8_t high = threshold >> 8;

    writeSingleByte(REGISTER_SP_TH_H_LSB, low);
    writeSingleByte(REGISTER_SP_TH_H_MSB, high);
}

unsigned int AS7341::getLowThreshold()
{
    uint8_t low = readSingleByte(REGISTER_SP_TH_L_LSB);
    uint8_t high = readSingleByte(REGISTER_SP_TH_L_MSB);
    return ((high << 8) | low);
}

unsigned int AS7341::getHighThreshold()
{
    uint8_t low = readSingleByte(REGISTER_SP_TH_H_LSB);
    uint8_t high = readSingleByte(REGISTER_SP_TH_H_MSB);
    return ((high << 8) | low);
}

void AS7341::setAPERS(uint8_t value)
{
    // Register value must be less than 15
    value &= 0x0f;

    // Get current APERS value and change bits 3..0 only
    uint8_t currentAPERS = getAPERS();
    currentAPERS &= 0x0f;
    currentAPERS |= value;
    // Write value back
    writeSingleByte(REGISTER_PERS, currentAPERS);
}

uint8_t AS7341::getAPERS()
{
    return readSingleByte(REGISTER_PERS);
}

void AS7341::enableMeasurements()
{
    setRegisterBit(REGISTER_STATUS, 1);
}

void AS7341::disableMeasurements()
{
    clearRegisterBit(REGISTER_STATUS, 1);
}

bool AS7341::isMeasurementEnabled()
{
    return isBitSet(REGISTER_STATUS, 1);
}

void AS7341::clearThresholdInterrupts()
{
    writeSingleByte(REGISTER_STATUS, 0xff);
}

void AS7341::enableThresholdInterrupt()
{
    writeSingleByte(REGISTER_CFG_12, 0);
    DelayMs(10);
    setRegisterBit(REGISTER_INTENAB, 3);
}

void AS7341::disableThresholdInterrupt()
{
    clearRegisterBit(REGISTER_INTENAB, 3);
}

bool AS7341::lowThresholdInterruptSet()
{
    bool lowSet = isBitSet(REGISTER_STATUS_3, 4);
    bool highSet = isBitSet(REGISTER_STATUS_3, 5);
    if (lowSet)
    {
        return (!highSet);
    }
    else
    {
        return false;
    }
}

bool AS7341::highThresholdInterruptSet()
{
    bool lowSet = isBitSet(REGISTER_STATUS_3, 4);
    bool highSet = isBitSet(REGISTER_STATUS_3, 5);
    if (highSet)
    {
        return (!lowSet);
    }
    else
    {
        return false;
    }
}

int AS7341::getFlickerFrequency()
{
    // Configure SMUX for flicker detection
    writeSingleByte(REGISTER_ENABLE, 0x01);
    writeSingleByte(REGISTER_CFG_9, 0x10);
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10);
    writeSingleByte(0x00, 0x00);
    writeSingleByte(0x01, 0x00);
    writeSingleByte(0x02, 0x00);
    writeSingleByte(0x03, 0x00);
    writeSingleByte(0x04, 0x00);
    writeSingleByte(0x05, 0x00);
    writeSingleByte(0x06, 0x00);
    writeSingleByte(0x07, 0x00);
    writeSingleByte(0x08, 0x00);
    writeSingleByte(0x09, 0x00);
    writeSingleByte(0x0A, 0x00);
    writeSingleByte(0x0B, 0x00);
    writeSingleByte(0x0C, 0x00);
    writeSingleByte(0x0D, 0x00);
    writeSingleByte(0x0E, 0x00);
    writeSingleByte(0x0F, 0x00);
    writeSingleByte(0x10, 0x00);
    writeSingleByte(0x11, 0x00);
    writeSingleByte(0x12, 0x00);
    writeSingleByte(0x13, 0x60);
    writeSingleByte(REGISTER_ENABLE, 0x13);

    // Set FD integration time to approx 2.84ms and gain to 32x (7:3 --> 6)
    // FD time is 0x3ff (maximum allowable integration time)
    writeSingleByte(REGISTER_FD_TIME_1, 0xff);
    writeSingleByte(REGISTER_FD_TIME_2, 0x1b);

    // Enable flicker detection
    setRegisterBit(REGISTER_ENABLE, 6);

    // Get resulting flicker status
    uint8_t flickerStatus = readSingleByte(REGISTER_FD_STATUS);

    if (isBitSet(REGISTER_FD_STATUS, 5))
    {
        // mask result
        flickerStatus &= 0x0c;
        // Shift right 2 bits
        flickerStatus = flickerStatus >> 2;

        switch (flickerStatus)
        {
        case 1:
            return 100;

        case 2:
            return 120;

        case 3:
        default:
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

void AS7341::setBankConfiguration(uint8_t regAddress)
{
    bool needsHighBank = (regAddress >= 0x60 && regAddress <= 0x74);
    if (needsHighBank == _currentBankIsHigh)
    {
        return;
    }

    uint8_t value = 0;
    HAL_I2C_Mem_Read(_hi2c, _addr, REGISTER_CFG_0, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);

    if (needsHighBank)
        value |= (1 << 4);
    else
        value &= ~(1 << 4);

    HAL_I2C_Mem_Write(_hi2c, _addr, REGISTER_CFG_0, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);

    _currentBankIsHigh = needsHighBank;
}

uint8_t AS7341::readSingleByte(uint8_t registerAddress)
{
    setBankConfiguration(registerAddress);
    uint8_t value = 0;
    HAL_I2C_Mem_Read(_hi2c, _addr << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
    return value;
}

void AS7341::writeSingleByte(uint8_t registerAddress, uint8_t const value)
{
    setBankConfiguration(registerAddress);
    HAL_I2C_Mem_Write(_hi2c, _addr << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&value, 1, 10);
}

void AS7341::readMultipleBytes(uint8_t registerAddress, uint8_t *buffer, uint8_t const packetLength)
{
    setBankConfiguration(registerAddress);
    HAL_I2C_Mem_Read(_hi2c, _addr << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, buffer, packetLength, 50);
}

void AS7341::writeMultipleBytes(uint8_t registerAddress, const uint8_t *buffer, uint8_t const packetLength)
{
    setBankConfiguration(registerAddress);
    HAL_I2C_Mem_Write(_hi2c, _addr << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer, packetLength, 50);
}

void AS7341::setRegisterBit(uint8_t registerAddress, uint8_t const bitPosition)
{
    uint8_t value = readSingleByte(registerAddress);
    value |= (1 << bitPosition);
    writeSingleByte(registerAddress, value);
}

void AS7341::clearRegisterBit(uint8_t registerAddress, uint8_t const bitPosition)
{
    uint8_t value = readSingleByte(registerAddress);
    value &= ~(1 << bitPosition);
    writeSingleByte(registerAddress, value);
}

bool AS7341::isBitSet(uint8_t registerAddress, uint8_t const bitPosition)
{
    uint8_t value = readSingleByte(registerAddress);
    return (value & (1 << bitPosition)) != 0;
}

void AS7341::writeSMUX(const uint8_t *smux_data)
{
    writeSingleByte(REGISTER_ENABLE, 0x01); // 保持待机状态
    writeSingleByte(REGISTER_CFG_9, 0x10);  // 允许配置 SMUX
    writeSingleByte(REGISTER_INTENAB, 0x01);
    writeSingleByte(REGISTER_CFG_6, 0x10); // 设为写入模式

    writeMultipleBytes(0x00, smux_data, 20);
    writeSingleByte(REGISTER_ENABLE, 0x11); // 执行 SMUX 连线指令
}

void AS7341::setMuxCustom(AS7341_CH adc0, AS7341_CH adc1, AS7341_CH adc2, AS7341_CH adc3, AS7341_CH adc4, AS7341_CH adc5)
{
    uint8_t smux[20] = {0}; // 初始状态：断开所有传感器的内部连线
    AS7341_CH channels[6] = {adc0, adc1, adc2, adc3, adc4, adc5};

    for (int i = 0; i < 6; i++)
    {
        if (channels[i] == AS7341_CH::NONE)
            continue;

        // 芯片内部 ADC 编号是 1 到 6 (1 代表 ADC0, 6 代表 ADC5)
        uint8_t adc_code = i + 1;
        switch (channels[i])
        {
        case AS7341_CH::F1:
            smux[1] |= adc_code;
            smux[16] |= adc_code;
            break;
        case AS7341_CH::F2:
            smux[5] |= adc_code;
            smux[12] |= (adc_code << 4);
            break;
        case AS7341_CH::F3:
            smux[0] |= (adc_code << 4);
            smux[15] |= (adc_code << 4);
            break;
        case AS7341_CH::F4:
            smux[5] |= (adc_code << 4);
            smux[13] |= adc_code;
            break;
        case AS7341_CH::F5:
            smux[6] |= (adc_code << 4);
            smux[9] |= (adc_code << 4);
            break;
        case AS7341_CH::F6:
            smux[4] |= adc_code;
            smux[14] |= (adc_code << 4);
            break;
        case AS7341_CH::F7:
            smux[7] |= adc_code;
            smux[10] |= adc_code;
            break;
        case AS7341_CH::F8:
            smux[3] |= (adc_code << 4);
            smux[14] |= adc_code;
            break;
        case AS7341_CH::CLEAR:
            smux[8] |= (adc_code << 4);
            smux[17] |= (adc_code << 4);
            break;
        case AS7341_CH::NIR:
            smux[19] |= adc_code;
            break;
        default:
            break;
        }
    }

    writeSMUX(smux);
}

AS7341_ERROR AS7341::readCustomChannels(uint16_t *results)
{
    setRegisterBit(REGISTER_ENABLE, 1);
    uint32_t start = GetSysTimeMs();
    while (!isBitSet(REGISTER_STATUS_2, 6))
    {
        if (GetSysTimeMs() - start > 5000)
            return AS7341_ERROR::MEASUREMENT_TIMEOUT;
    }

    uint8_t buffer[12] = {0};
    uint8_t bytes_to_read = 6 * 2;
    readMultipleBytes(REGISTER_CH0_DATA_L, buffer, bytes_to_read);
    for (int i = 0; i < 6; i++)
    {
        results[i] = (buffer[i * 2 + 1] << 8) | buffer[i * 2];
    }

    return AS7341_ERROR::OK;
}