#include "serial_servo.h"
#include "usart.h"
#include <string.h>
#include "utils.h"

/* 全局变量 */
SerialServoControllerTypeDef serial_servo_controller;
volatile bool is_serial_servo_tx_complete = false;
volatile bool is_serial_servo_rx_complete = false;

static int SerialWriteAndRead(SerialServoControllerTypeDef *self, SerialServoCmdTypeDef *frame, bool tx_only)
{
    is_serial_servo_tx_complete = false;
    is_serial_servo_rx_complete = false;

    memcpy(&self->tx_frame, frame, sizeof(SerialServoCmdTypeDef));
    self->tx_only = tx_only;
    self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;

    if (!self->tx_only)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart8);
        __HAL_UART_CLEAR_FEFLAG(&huart8);
        __HAL_UART_CLEAR_NEFLAG(&huart8);
        __HAL_UART_CLEAR_PEFLAG(&huart8);
        __HAL_UART_CLEAR_IDLEFLAG(&huart8);
        __HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(&huart8, UART_IT_ERR);
        
        // HAL_UART_AbortReceive(&huart8);
        // HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_rx_buffer, UART8_RX_BUF_SIZE);
        // __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    }

    if (HAL_UART_Transmit_DMA(&huart8, (uint8_t *)&self->tx_frame, self->tx_frame.elements.length + 3) != HAL_OK)
    {
        return -1;
    }

    uint32_t start_ms = GetSysTimeMs();

    while (!is_serial_servo_tx_complete)
    {
        if ((GetSysTimeMs() - start_ms) >= self->proc_timeout)
        {
            return -1;
        }
        DelayUs(1);
    }

    if (!self->tx_only)
    {
        start_ms = GetSysTimeMs();
        while (!is_serial_servo_rx_complete)
        {
            if ((GetSysTimeMs() - start_ms) >= self->proc_timeout)
            {
                __HAL_UART_DISABLE_IT(&huart8, UART_IT_RXNE);
                __HAL_UART_DISABLE_IT(&huart8, UART_IT_ERR);
                return -1;
            }
            DelayUs(1);
        }
        __HAL_UART_DISABLE_IT(&huart8, UART_IT_RXNE);
        __HAL_UART_DISABLE_IT(&huart8, UART_IT_ERR);
    }

    return 0;
}

void SerialServoRxCpltCallback(SerialServoControllerTypeDef *self, uint8_t *data, uint16_t data_size)
{
    /** #TODO: complete this */
    if (data_size <6)
    {
        return;
    }

    if (data[0] == SERIAL_SERVO_FRAME_HEADER && data[1] == SERIAL_SERVO_FRAME_HEADER)
    {
        uint8_t length = data[3];
        uint16_t frame_len = (uint16_t)length + 3;
        if (frame_len == data_size)
        {
            uint8_t crc = serial_servo_checksum(data);
            if (crc == data[frame_len - 1])
            {
                memcpy(&self->rx_frame, data, frame_len);
                self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
                is_serial_servo_rx_complete = true;
            }
        }
    }
}

void SerialServoInit(void)
{
    serial_servo_controller_object_init(&serial_servo_controller);
    serial_servo_controller.proc_timeout = 4;
    serial_servo_controller.serial_write_and_read = SerialWriteAndRead;
}

void SerialServoSetPosDeg(SerialServoControllerTypeDef *self, uint32_t servo_id, float pos_deg, uint32_t duration_ms)
{
    if (pos_deg < 0.0f)
        pos_deg = 0.0f;
    if (pos_deg > 120.0f)
        pos_deg = 120.0f;

    int position = (int)(pos_deg * (1000.0f / 120.0f) + 0.5f);
    if (position < 0)
        position = 0;
    if (position > 1000)
        position = 1000;

    serial_servo_set_position(self, servo_id, position, duration_ms);
}