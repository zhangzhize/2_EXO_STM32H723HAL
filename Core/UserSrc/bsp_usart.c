#include <stdio.h>
#include <string.h>
#include "bsp_usart.h"

#include "usbd_cdc_if.h"
#define UART_PRINTF_HANDLE   huart9    /*!< printf 备用硬件 UART 句柄 */
#define UART_PRINTF_TIMEOUT  100       /*!< UART 发送超时 (ms) */
extern USBD_HandleTypeDef hUsbDeviceHS;

/* 单例回调上下文：保存 Exo 实例指针，供中断回调使用 */
static void *s_usart_ctx = NULL;
/* 单例接收回调函数指针 */
static BspUartRxCallback s_usart_rx_cb = NULL;
/* 单例错误回调函数指针 */
static BspUartErrorCallback s_usart_err_cb = NULL;

/**
 * @brief  GCC 环境 printf 重定向 —— 通过 USB CDC 发送
 * @param  file  文件描述符（未使用）
 * @param  ptr   数据缓冲区
 * @param  len   数据长度
 * @return 实际发送的字节数
 *
 * @note   当前优先使用 USB CDC 虚拟串口输出。
 *         备用的 HAL_UART_Transmit 代码已注释，可按需切换。
 *         超时机制：最多等待 0xFFFF 次 USBD_BUSY 后放弃。
 */
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
/* 如果使用了 MicroLIB，这段 struct 定义可以省略 */
struct __FILE { int handle; };
FILE __stdout;

/**
 * @brief  ARMCC (Keil AC6/MicroLIB) 环境 printf 重定向 —— 通过 USB CDC 逐字符发送
 * @param  ch  要输出的字符
 * @param  f   文件指针
 * @return 输出的字符
 */
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

/**
 * @brief  注册 UART 接收回调
 * @param  ctx  Exo 对象指针
 * @param  cb   接收回调函数
 */
void BspUsartRegisterRxCallback(void *ctx, BspUartRxCallback cb)
{
    s_usart_ctx = ctx;
    s_usart_rx_cb = cb;
}

/**
 * @brief  注册 UART 错误回调
 * @param  ctx  Exo 对象指针
 * @param  cb   错误回调函数
 */
void BspUsartRegisterErrorCallback(void *ctx, BspUartErrorCallback cb)
{
    s_usart_ctx = ctx;
    s_usart_err_cb = cb;
}

/**
 * @brief  HAL UART 扩展接收事件回调 —— 单例模式转发点
 *
 * @details 当 DMA 接收半满 (HT)、全满 (TC) 或空闲中断 (IDLE) 发生时，
 *          HAL 库调用此 weak 函数。本模块将其转发到已注册的 Exo 回调。
 *          上层在回调中处理已接收的数据并重新启动 DMA 接收。
 *
 * @param  huart UART 句柄
 * @param  size  当前 DMA 已传输的字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (s_usart_rx_cb != NULL)
    {
        s_usart_rx_cb(s_usart_ctx, huart, size);
    }
}

/**
 * @brief  HAL UART 错误回调 —— 错误事件转发
 *
 * @details 当发生帧错误(FE)、噪声错误(NE)、溢出错误(ORE)等时，
 *          HAL 库调用此 weak 函数。本模块将其转发到上层 Exo 对象，
 *          由上层决定是否重试、复位或报警。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_usart_err_cb != NULL)
    {
        s_usart_err_cb(s_usart_ctx, huart);
    }
}
