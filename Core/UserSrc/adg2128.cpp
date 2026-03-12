#include "adg2128.hpp"

static const uint8_t readback_addr[12] = {0x34, 0x3c, 0x74, 0x7c, 0x35, 0x3d, 0x75, 0x7d, 0x36, 0x3e, 0x76, 0x7e};

const char *const ADG2128::errorToStr(ADG2128_ERROR err)
{
    switch (err)
    {
    case ADG2128_ERROR::NO_ERROR:   return "NO_ERROR";
    case ADG2128_ERROR::ABSENT:     return "ABSENT";
    case ADG2128_ERROR::BUS:        return "BUS";
    case ADG2128_ERROR::BAD_COLUMN: return "BAD_COLUMN";
    case ADG2128_ERROR::BAD_ROW:    return "BAD_ROW";
    default:                        return "UNKNOWN";
    }
}

ADG2128::ADG2128()
{
}

ADG2128::~ADG2128()
{
    // 析构时直接拉低复位引脚进行保护
    if ((_rst_port != nullptr) && (_rst_pin != 255))
    {
        HAL_GPIO_WritePin(_rst_port, _rst_pin, GPIO_PIN_RESET);
    }
}

ADG2128_ERROR ADG2128::begin(I2C_HandleTypeDef *hi2c, uint8_t addr_offset, GPIO_TypeDef *rst_port, uint16_t rst_pin)
{
    if (hi2c != nullptr) {
        _hi2c = hi2c;
    }
    if (rst_port != nullptr && rst_pin != 255) {
        _rst_port = rst_port;
        _rst_pin = rst_pin;
    }
    _addr = BASE_ADDR + addr_offset;

    return reset();
}

ADG2128_ERROR ADG2128::reset()
{
    if (_rst_port != nullptr && _rst_pin != 255)
    {
        HAL_GPIO_WritePin(_rst_port, _rst_pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(_rst_port, _rst_pin, GPIO_PIN_SET);
        HAL_Delay(10);

        for (int i = 0; i < 12; i++) {
            _values[i] = 0;
        }
    }
    else
    {
        // 没有硬件复位引脚时，用软件 I2C 逐个断开 96 个开关
        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                if (ADG2128_ERROR::NO_ERROR != unsetRoute(j, i, !((11 == i) && (7 == j))))
                {
                    return ADG2128_ERROR::BUS;
                }
            }
        }
    }
    
    return (0 == readDevice()) ? ADG2128_ERROR::NO_ERROR : ADG2128_ERROR::BUS;
}

ADG2128_ERROR ADG2128::composeFirstByte(uint8_t col, uint8_t row, bool set, uint8_t *result)
{
    if (col > 7)  return ADG2128_ERROR::BAD_COLUMN;
    if (row > 11) return ADG2128_ERROR::BAD_ROW;
    uint8_t temp = row;
    if (temp >= 6) temp = temp + 2;
    *result = (temp << 3) + col + (set ? 0x80 : 0x00);
    return ADG2128_ERROR::NO_ERROR;
}

ADG2128_ERROR ADG2128::setRoute(uint8_t col, uint8_t row, bool defer)
{
    return changeRoute(col, row, true, defer);
}

ADG2128_ERROR ADG2128::unsetRoute(uint8_t col, uint8_t row, bool defer)
{
    return changeRoute(col, row, false, defer);
}

ADG2128_ERROR ADG2128::changeRoute(uint8_t col, uint8_t row, bool sw_closed, bool defer)
{
    uint8_t temp;
    ADG2128_ERROR return_value = composeFirstByte(col, row, sw_closed, &temp);
    if (ADG2128_ERROR::NO_ERROR == return_value)
    {
        return_value = ADG2128_ERROR::BUS;
        if (0 == writeDevice(temp, (defer ? 0 : 1)))
        {
            _values[row] = (sw_closed) ? (_values[row] | (1 << col)) : (_values[row] & ~(1 << col));
            return_value = ADG2128_ERROR::NO_ERROR;
        }
    }
    return return_value;
}

ADG2128_ERROR ADG2128::refresh()
{
    return (0 == readDevice()) ? ADG2128_ERROR::NO_ERROR : ADG2128_ERROR::BUS;
}

uint8_t ADG2128::getCols(uint8_t row)
{
    if (row > 11) return 0;
    return _values[row];
}

uint16_t ADG2128::getRows(uint8_t col)
{
    if (col > 7) return 0;
    uint16_t ret = 0;
    for (uint8_t i = 0; i < 12; i++)
    {
        uint8_t val = (_values[i] >> col) & 1;
        ret |= (val << i);
    }
    return ret;
}

int8_t ADG2128::readDevice()
{
    if (_hi2c == nullptr) 
    {
        return -3; 
    }
    
    for (uint8_t row = 0; row < 12; row++)
    {
        uint8_t rx_buf[2] = {0, 0};
        if (HAL_I2C_Mem_Read(_hi2c, (_addr << 1), readback_addr[row], I2C_MEMADD_SIZE_8BIT, rx_buf, 2, 100) == HAL_OK)
        {
            _values[row] = rx_buf[1];
        }
        else
        {
            return -2;
        }
    }
    return 0;
}

int8_t ADG2128::writeDevice(uint8_t row, uint8_t conn)
{
    if (_hi2c == nullptr) 
    {
        return -3; 
    }

    if (HAL_I2C_Mem_Write(_hi2c, (_addr << 1), row, I2C_MEMADD_SIZE_8BIT, &conn, 1, 100) == HAL_OK)
    {
        return 0;
    }
    else
    {
        return -2;
    }
}