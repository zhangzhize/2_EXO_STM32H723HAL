#include <stdio.h>
#include <string.h>
#include "bsp_usart.h"

#include "usbd_cdc_if.h"
#define UART_PRINTF_HANDLE   huart9
#define UART_PRINTF_TIMEOUT  100
extern USBD_HandleTypeDef hUsbDeviceHS;

static void *s_usart_ctx = NULL;
static BspUartRxCallback s_usart_rx_cb = NULL;
static BspUartErrorCallback s_usart_err_cb = NULL;

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

void BspUsartRegisterRxCallback(void *ctx, BspUartRxCallback cb)
{
    s_usart_ctx = ctx;
    s_usart_rx_cb = cb;
}

void BspUsartRegisterErrorCallback(void *ctx, BspUartErrorCallback cb)
{
    s_usart_ctx = ctx;
    s_usart_err_cb = cb;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (s_usart_rx_cb != NULL)
    {
        s_usart_rx_cb(s_usart_ctx, huart, size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_usart_err_cb != NULL)
    {
        s_usart_err_cb(s_usart_ctx, huart);
    }
}

