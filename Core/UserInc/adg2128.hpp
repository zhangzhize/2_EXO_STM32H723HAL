#ifndef ADG2128_HPP
#define ADG2128_HPP

#include "main.h"

enum class ADG2128_ERROR : int8_t
{
  NO_ERROR           = 0,   // There was no error.
  ABSENT             = -1,  // The ADG2128 appears to not be connected to the bus.
  BUS                = -2,  // Something went wrong with the i2c bus.
  BAD_COLUMN         = -3,  // Column was out-of-bounds.
  BAD_ROW            = -4   // Row was out-of-bounds.
};

class ADG2128
{
public:
    ADG2128();
    ~ADG2128();

    ADG2128_ERROR begin(I2C_HandleTypeDef* hi2c, uint8_t addr_offset = 0, GPIO_TypeDef* rst_port= nullptr, uint16_t rst_pin = 255);

    ADG2128_ERROR reset();     
    ADG2128_ERROR refresh();   
    ADG2128_ERROR changeRoute(uint8_t col, uint8_t row, bool sw_closed, bool defer);
    ADG2128_ERROR setRoute(uint8_t col, uint8_t row, bool defer = false);
    ADG2128_ERROR unsetRoute(uint8_t col, uint8_t row, bool defer = false);

    uint8_t  getCols(uint8_t row);
    uint16_t getRows(uint8_t col);

    static const char* const errorToStr(ADG2128_ERROR);

private:
    uint8_t _addr = 0;
    I2C_HandleTypeDef* _hi2c = nullptr;    
    GPIO_TypeDef* _rst_port  = nullptr; 
    uint16_t _rst_pin = 255;  

    uint8_t  _values[12] = {0};

    ADG2128_ERROR composeFirstByte(uint8_t col, uint8_t row, bool set, uint8_t* result);
    int8_t readDevice();
    int8_t writeDevice(uint8_t row, uint8_t conn);

    static constexpr uint8_t  BASE_ADDR = 0x70;
};

#endif
