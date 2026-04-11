#include "mag_encoder.hpp"
#include "gpio.h"

__attribute__((section(".dma_buf"), aligned(32))) uint8_t g_mag_encoder_tx_dma_buf[2][32] = {
    {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0B},
    {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0B}
};
__attribute__((section(".dma_buf"), aligned(32))) uint8_t g_mag_encoder_rx_dma_buf[2][32] = {0};

uint8_t MagEncoder::instance_count = 0;

/** 目前只能定义一个示例, 因为多个实例会导致上面的内存访问冲突; 可以用二维数组instance_cnt_的方式 */
MagEncoder::MagEncoder(UART_HandleTypeDef &huart) : huart_(huart)
{
    if (instance_count < 2)
    {
        rx_buffer_ = g_mag_encoder_rx_dma_buf[instance_count];
        tx_buffer_ = g_mag_encoder_tx_dma_buf[instance_count];
        instance_count++;
    }
}

void MagEncoder::SendRequest()
{
    if (state_ != State::kIdle) return;

    if (is_first_reading_)
    {
        is_first_reading_ = false;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart_, rx_buffer_, 32);
    }
    HAL_UART_Transmit_DMA(&huart_, tx_buffer_, 8);
    state_ = State::kWaitingForData;
}


void MagEncoder::UartRxCallback(UART_HandleTypeDef *huart, const uint8_t* data, uint16_t data_size)
{
    if (data == nullptr || data_size < 7 || huart != &huart_) return;

    raw_position_reading_ = HexArrayToDec(data, data_size);
    scaled_offsetted_position_ = (float)raw_position_reading_ / scale_factor_;
    state_ = State::kIdle;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart_, rx_buffer_, 32);
}

uint32_t MagEncoder::HexArrayToDec(const uint8_t* hex_array, uint8_t size)
{
    if (size < 7) return 0;

    uint32_t result = 0;
    uint8_t ch[4] = {hex_array[3], hex_array[4], hex_array[5], hex_array[6]};
    for(uint8_t i = 0; i < 4; i++)
    {
        result = (result << 8) | ch[i];
    }
    return result;
}
