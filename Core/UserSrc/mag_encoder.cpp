#include "mag_encoder.hpp"
#include "gpio.h"

__attribute__((section(".dma_buf"), aligned(32))) uint8_t g_mag_encoder_tx_dma_buf[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0B};
__attribute__((section(".dma_buf"), aligned(32))) uint8_t g_mag_encoder_rx_dma_buf[32] = {0};

/** 目前只能定义一个示例, 因为多个实例会导致上面的内存访问冲突; 可以用二维数组instance_cnt_的方式 */
MagEncoder::MagEncoder(UART_HandleTypeDef* huart, GPIO_TypeDef* de_port, uint16_t de_pin) : huart_(huart), de_port_(de_port), de_pin_(de_pin)
{
    ptr_rxbuffer_ = g_mag_encoder_rx_dma_buf;
    ptr_txbuffer_ = g_mag_encoder_tx_dma_buf;
}

void MagEncoder::SendRequest()
{
    if (state_ != State::kIdle) return;

    SetTxMode();
    HAL_UART_Transmit_DMA(huart_, g_mag_encoder_tx_dma_buf, 8);
    state_ = State::kSending;
}

/** 需要在发送完成回调函数中调用下面这个函数 */
void MagEncoder::TxCptCallback()
{
    if (state_ != State::kSending) return;

    SetRxMode();
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, g_mag_encoder_rx_dma_buf, sizeof(g_mag_encoder_rx_dma_buf));
    state_ = State::kReceiving;
}

void MagEncoder::SetTxMode()
{
    if (de_port_ == nullptr) return;

    HAL_GPIO_WritePin(de_port_, de_pin_, GPIO_PIN_SET);
}

void MagEncoder::SetRxMode()
{
    if (de_port_ == nullptr) return;

    HAL_GPIO_WritePin(de_port_, de_pin_, GPIO_PIN_RESET);
}

/** 需要在接收完成回调函数中调用下面这个函数 */
void MagEncoder::ProcessData(uint8_t* data, uint16_t data_size)
{
    if (data == nullptr || data_size < 7) return;

    if (state_ != State::kReceiving) return;

    raw_position_reading_ = HexArrayToDec(data, data_size);
    scaled_offsetted_position_ = ((float)raw_position_reading_ / scale_factor_) - (float)scaled_zero_offset_;
    state_ = State::kIdle;
}

float MagEncoder::GetPosition() const
{
    return scaled_offsetted_position_;
}

void MagEncoder::SetZeroOffset(uint32_t new_offset)
{
    scaled_zero_offset_ = new_offset;
}

uint32_t MagEncoder::HexArrayToDec(uint8_t* hex_array, uint8_t size)
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

