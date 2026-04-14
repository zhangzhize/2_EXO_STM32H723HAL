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
        kWaitingForData,
    };

    explicit MagEncoder(UART_HandleTypeDef &uart);
    virtual ~MagEncoder() = default;

    void SendRequest();
    void UartRxCallback(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t data_size);

    float scaled_position_ = 0.0f;
    static uint16_t Crc16Modbus(const uint8_t *data, uint16_t data_size);
    static uint32_t HexArrayToDec(const uint8_t *hex_array, uint8_t length);
    State state_ = State::kIdle;

    UART_HandleTypeDef &huart_;
private:

    uint8_t *rx_buffer_ = nullptr;
    uint8_t *tx_buffer_ = nullptr;

    float scale_factor_ = 10.0f;
    uint32_t raw_position_reading_ = 0;

    // State state_ = State::kIdle;

    static uint8_t instance_count;

    bool is_first_reading_ = true;
};

#endif