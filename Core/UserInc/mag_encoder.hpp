#ifndef MAG_ENCODER_HPP
#define MAG_ENCODER_HPP

#include <cstdint>
#include "usart.h"

class MagEncoder
{
public:
    enum class State : uint8_t
    {
        kIdle,
        kSending,
        kReceiving
    };

    explicit MagEncoder(UART_HandleTypeDef* huart, GPIO_TypeDef* de_port, uint16_t de_pin);
    virtual ~MagEncoder() = default;

    void SendRequest();
    void TxCptCallback();

    void ProcessData(uint8_t* data, uint16_t data_size);
    float GetPosition() const;
    void SetZeroOffset(uint32_t new_offset);

private:
    static uint32_t HexArrayToDec(uint8_t* hex_array, uint8_t length);
    void SetTxMode();
    void SetRxMode();
    
    UART_HandleTypeDef* huart_;
    GPIO_TypeDef* de_port_;
    uint16_t de_pin_;

    uint8_t* ptr_rxbuffer_ = nullptr;
    uint8_t* ptr_txbuffer_ = nullptr;
    
    float scale_factor_ = 10.0f;
    uint32_t scaled_zero_offset_ = 2000000;
    uint32_t raw_position_reading_ = 0;
    float scaled_offsetted_position_ = 0.0f;

    State state_ = State::kIdle;
};

#endif