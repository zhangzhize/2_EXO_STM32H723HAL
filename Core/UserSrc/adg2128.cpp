#include "adg2128.hpp"

static const uint8_t readbackaddr_[12] = {0x34, 0x3c, 0x74, 0x7c, 0x35, 0x3d, 0x75, 0x7d, 0x36, 0x3e, 0x76, 0x7e};

const char *const ADG2128::ErrorToString(ADG2128_ERROR err)
{
    switch (err)
    {
    case ADG2128_ERROR::OK:              return "OK";
    case ADG2128_ERROR::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case ADG2128_ERROR::INIT_ERROR:      return "INIT_ERROR";

    case ADG2128_ERROR::ABSENT:          return "ABSENT";
    case ADG2128_ERROR::BUS_ERROR:      return "BUS_ERROR";

    case ADG2128_ERROR::INVALID_COLUMN:  return "INVALID_COLUMN";
    case ADG2128_ERROR::INVALID_ROW:     return "INVALID_ROW";
    default:                             return "UNKNOWN";
    }
}

ADG2128::ADG2128() {}

ADG2128::~ADG2128()
{
    if ((rst_port_ != nullptr) && (rst_pin_ != kNoPin))
    {
        HAL_GPIO_WritePin(rst_port_, rst_pin_, GPIO_PIN_RESET);
    }
}

ADG2128_ERROR ADG2128::Begin(I2C_HandleTypeDef *hi2c, uint8_t addr_offset, GPIO_TypeDef *rst_port, uint16_t rst_pin)
{
    if (hi2c == nullptr)
    {
        return ADG2128_ERROR::INIT_ERROR;
    }

    hi2c_ = hi2c;
    addr_ = kBaseAddr + addr_offset;

    if (rst_port != nullptr && rst_pin != kNoPin)
    {
        rst_port_ = rst_port;
        rst_pin_ = rst_pin;
        
    }

    if (HAL_I2C_IsDeviceReady(hi2c_, (addr_ << 1), 2, 100) != HAL_OK)
    {
        return ADG2128_ERROR::ABSENT;
    }

    return Reset();
}

ADG2128_ERROR ADG2128::Reset()
{
    if (!IsInitialized())
    {
        return ADG2128_ERROR::NOT_INITIALIZED;
    }

    if (rst_port_ != nullptr && rst_pin_ != kNoPin)
    {
        HAL_GPIO_WritePin(rst_port_, rst_pin_, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(rst_port_, rst_pin_, GPIO_PIN_SET);
        HAL_Delay(10);

        for (int i = 0; i < 12; i++)
        {
            values_[i] = 0;
        }
    }
    else
    {
        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                ADG2128_ERROR e = UnsetRoute(j, i, !((11 == i) && (7 == j)));
                if (e != ADG2128_ERROR::OK)
                {
                    return e;
                }
            }
        }
    }

    return ReadDevice();
}

ADG2128_ERROR ADG2128::ComposeFirstByte(uint8_t col, uint8_t row, bool set, uint8_t *result)
{
    if (col > 7)  return ADG2128_ERROR::INVALID_COLUMN;
    if (row > 11) return ADG2128_ERROR::INVALID_ROW;
    uint8_t temp = row;
    if (temp >= 6) temp = temp + 2;
    *result = (temp << 3) + col + (set ? 0x80 : 0x00);
    return ADG2128_ERROR::OK;
}

ADG2128_ERROR ADG2128::SetRoute(uint8_t col, uint8_t row, bool store_only)
{
    return ChangeRoute(col, row, true, store_only);
}

ADG2128_ERROR ADG2128::UnsetRoute(uint8_t col, uint8_t row, bool store_only)
{
    return ChangeRoute(col, row, false, store_only);
}

ADG2128_ERROR ADG2128::ChangeRoute(uint8_t col, uint8_t row, bool sw_closed, bool store_only)
{
    if (!IsInitialized())
    {
        return ADG2128_ERROR::NOT_INITIALIZED;
    }

    uint8_t temp;
    ADG2128_ERROR ret = ComposeFirstByte(col, row, sw_closed, &temp);
    if (ADG2128_ERROR::OK == ret)
    {
        ret = WriteDevice(temp, (store_only ? 0 : 1));
        if (ADG2128_ERROR::OK == ret)
        {
            values_[row] = (sw_closed) ? (values_[row] | (1 << col)) : (values_[row] & ~(1 << col));
        }
    }
    return ret;
}

ADG2128_ERROR ADG2128::Refresh()
{
    return ReadDevice();
}

uint8_t ADG2128::GetCols(uint8_t row)
{
    if (row > 11) return 0;
    return values_[row];
}

uint16_t ADG2128::GetRows(uint8_t col)
{
    if (col > 7) return 0;
    uint16_t ret = 0;
    for (uint8_t i = 0; i < 12; i++)
    {
        uint8_t val = (values_[i] >> col) & 1;
        ret |= (val << i);
    }
    return ret;
}

ADG2128_ERROR ADG2128::ReadDevice()
{
    if (!IsInitialized())
    {
        return ADG2128_ERROR::NOT_INITIALIZED;
    }

    for (uint8_t row = 0; row < 12; row++)
    {
        uint8_t rx_buf[2] = {0, 0};
        if (HAL_I2C_Mem_Read(hi2c_, (addr_ << 1), readbackaddr_[row], I2C_MEMADD_SIZE_8BIT, rx_buf, 2, 100) == HAL_OK)
        {
            values_[row] = rx_buf[1];
        }
        else
        {
            return ADG2128_ERROR::BUS_ERROR;
        }
    }

    return ADG2128_ERROR::OK;
}

ADG2128_ERROR ADG2128::WriteDevice(uint8_t row, uint8_t conn)
{
    if (!IsInitialized())
    {
        return ADG2128_ERROR::NOT_INITIALIZED;
    }

    if (HAL_I2C_Mem_Write(hi2c_, (addr_ << 1), row, I2C_MEMADD_SIZE_8BIT, &conn, 1, 100) == HAL_OK)
    {
        return ADG2128_ERROR::OK;
    }

    return ADG2128_ERROR::BUS_ERROR;
}