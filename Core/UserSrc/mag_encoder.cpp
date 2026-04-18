#include "mag_encoder.hpp"
#include "gpio.h"
#include "utils.h"

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
    if (state_ == State::kWaitingForData)
    {
        if (GetSysTimeMs() - request_start_ms_ > kTimeoutMs)
        {
            HAL_UART_AbortReceive(&huart_);
            state_ = State::kIdle;
        }
        else
        {
            return;
        }
    }

    if (state_ != State::kIdle) return;

    __HAL_UART_CLEAR_OREFLAG(&huart_);
    __HAL_UART_CLEAR_IDLEFLAG(&huart_);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart_, rx_buffer_, 32);

    tx_buffer_[0] = 0x01;
    tx_buffer_[1] = 0x03;
    tx_buffer_[2] = 0x00;
    tx_buffer_[3] = 0x00;
    tx_buffer_[4] = 0x00;
    tx_buffer_[5] = 0x02;
    tx_buffer_[6] = 0xc4;
    tx_buffer_[7] = 0x0B;
    HAL_UART_Transmit_DMA(&huart_, tx_buffer_, 8);
    request_start_ms_ = GetSysTimeMs();

    state_ = State::kWaitingForData;
}

void MagEncoder::UartRxCallback(UART_HandleTypeDef *huart, const uint8_t* data, uint16_t data_size)
{
    if (data == nullptr || data_size != 9 || huart != &huart_) return;

    uint16_t crc_received = data[8] << 8 | data[7];
    uint16_t crc_calculated = Crc16Modbus(data, 7);
    if (crc_received != crc_calculated) return;

    raw_position_reading_um_ = HexArrayToDec(data, data_size);
    absolute_position_mm_ = (float)raw_position_reading_um_ * kUm2Mm;
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart_, rx_buffer_, 32);
    state_ = State::kIdle;
}

uint16_t MagEncoder::Crc16Modbus(const uint8_t *data, uint16_t data_size)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < data_size; i++)
    {
        crc ^= data[i];

        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
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
