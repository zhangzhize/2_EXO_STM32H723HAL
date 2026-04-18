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

    float absolute_position_mm_ = 0.0f;
    static uint16_t Crc16Modbus(const uint8_t *data, uint16_t data_size);
    static uint32_t HexArrayToDec(const uint8_t *hex_array, uint8_t length);
    State state_ = State::kIdle;

    UART_HandleTypeDef &huart_;
private:

    uint8_t *rx_buffer_ = nullptr;
    uint8_t *tx_buffer_ = nullptr;

    static constexpr float kUm2Mm = 0.001f; 
    static constexpr uint32_t kTimeoutMs = 20;
    uint32_t raw_position_reading_um_ = 0;
    uint32_t request_start_ms_ = 0;

    // State state_ = State::kIdle;

    static uint8_t instance_count;

    bool is_first_reading_ = true;
};

#endif