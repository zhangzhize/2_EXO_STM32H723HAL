#include <stdio.h>
#include <string.h>
#include "bsp_usart.h"

__attribute__((section(".dma_buf"), aligned(32))) uint8_t uart8_rx_buffer[UART8_RX_BUF_SIZE];
__attribute__((section(".dma_buf"), aligned(32))) uint8_t uart9_rx_buffer[UART9_RX_BUF_SIZE];

#include "usbd_cdc_if.h"
#define UART_PRINTF_HANDLE   huart9
#define UART_PRINTF_TIMEOUT  100
extern USBD_HandleTypeDef hUsbDeviceHS;
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED)
    {
        uint8_t usb_status = USBD_OK;
        volatile uint32_t timeout = 0xFFFF;
        do
        {
            usb_status = CDC_Transmit_HS((uint8_t *)ptr, len);
            timeout--;
        } while (usb_status == USBD_BUSY && timeout > 0);
    }
    // HAL_UART_Transmit(&UART_PRINTF_HANDLE, (uint8_t *)ptr, len, UART_PRINTF_TIMEOUT);
    return len;
}

#else
/** 如果使用了 MicroLIB，这段 struct 定义可以省略 */
struct __FILE { int handle; };
FILE __stdout;

int fputc(int ch, FILE *f)
{
    if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED)
    {
        uint8_t usb_status = USBD_OK;
        do {
            usb_status = CDC_Transmit_HS((uint8_t *)&ch, 1);
        } while(usb_status == USBD_BUSY);
    }
    // HAL_UART_Transmit(&UART_PRINTF_HANDLE, (uint8_t *)&ch, 1, UART_PRINTF_TIMEOUT);
    return ch;
}
#endif

#include "usbd_cdc_if.h"
extern struct Exo *g_exo;
extern void CallExoUartRxCallback(struct Exo *ptr_exo, UART_HandleTypeDef *huart, uint16_t data_size);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UART8)
    {
        /** uncomment to debug */
        // uint8_t buf[128];
        // memcpy(buf, uart8_rx_buffer, size);
        // CDC_Transmit_HS(buf, size);
        if (size <= UART8_RX_BUF_SIZE)
        {
            CallExoUartRxCallback(g_exo, huart, size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_rx_buffer, UART8_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == UART9)
    {
        /** uncomment to debug */
        // uint8_t buf[128];
        // memcpy(buf, uart9_rx_buffer, size);
        // CDC_Transmit_HS(buf, size);
        if (size <= UART9_RX_BUF_SIZE)
        {
            CallExoUartRxCallback(g_exo, huart, size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uart9_rx_buffer, UART9_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);
    }
    else
    {
        /** MagEncoder内部会调用HAL_UARTEx_ReceiveToIdle_DMA */
        CallExoUartRxCallback(g_exo, huart, size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART8)
    {
        // uint32_t err_code = huart->ErrorCode;
        // printf("UART8 Error: %08x\r\n", err_code);
        // memset(uart8_rx_buffer, 0, UART8_RX_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_rx_buffer, UART8_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == UART9)
    {
        // memset(uart9_rx_buffer, 0, UART9_RX_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, uart9_rx_buffer, UART9_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);
    }
    /** #TODO 暂时认为MagEncoder的串口不会出错 */
}

