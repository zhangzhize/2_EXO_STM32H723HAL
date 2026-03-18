#ifndef ADG2128_HPP
#define ADG2128_HPP

#include "main.h"
#include <cstdint>

enum class ADG2128_ERROR : int8_t
{
    OK               = 0,    // There was no error.
    NOT_INITIALIZED  = -1,   // Begin() has not been called (or failed to set up the driver).
    INIT_ERROR       = -2,   // Initialization error.
    ABSENT           = -10,  // The ADG2128 appears to not be connected to the bus.
    BUS_ERROR        = -11,  // Something went wrong with the I2C bus.
    INVALID_COLUMN   = -20,  // Column was out-of-bounds.
    INVALID_ROW      = -21,  // Row was out-of-bounds.
};

class ADG2128
{
public:
    ADG2128();
    ~ADG2128();

    ADG2128_ERROR Begin(I2C_HandleTypeDef* hi2c, uint8_t addr_offset = 0, GPIO_TypeDef* rst_port= nullptr, uint16_t rst_pin = kNoPin);

    ADG2128_ERROR Reset();
    ADG2128_ERROR Refresh();
    ADG2128_ERROR ChangeRoute(uint8_t col, uint8_t row, bool sw_closed, bool store_only);
    ADG2128_ERROR SetRoute(uint8_t col, uint8_t row, bool store_only = false);
    ADG2128_ERROR UnsetRoute(uint8_t col, uint8_t row, bool store_only = false);

    uint8_t  GetCols(uint8_t row);
    uint16_t GetRows(uint8_t col);

    static const char* const ErrorToString(ADG2128_ERROR);
    enum class ApplyMode {
        kImmediate,
        kStoreOnly
    };

private:
    static constexpr uint8_t  kBaseAddr = 0x70;
    static constexpr uint16_t kNoPin    = 0xFFFF;

    uint8_t addr_ = 0;
    I2C_HandleTypeDef* hi2c_ = nullptr;
    GPIO_TypeDef* rst_port_  = nullptr;
    uint16_t rst_pin_ = kNoPin;

    uint8_t  values_[12] = {0};

    ADG2128_ERROR ComposeFirstByte(uint8_t col, uint8_t row, bool set, uint8_t* result);
    ADG2128_ERROR ReadDevice();
    ADG2128_ERROR WriteDevice(uint8_t row, uint8_t conn);

    bool IsInitialized() const { return hi2c_ != nullptr; }
};

#endif
