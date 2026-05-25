#ifndef BSP_USART_H
#define BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

typedef void (*BspUartRxCallback)(void *ctx, UART_HandleTypeDef *huart, uint16_t data_size);
typedef void (*BspUartErrorCallback)(void *ctx, UART_HandleTypeDef *huart);

void BspUsartRegisterRxCallback(void *ctx, BspUartRxCallback cb);
void BspUsartRegisterErrorCallback(void *ctx, BspUartErrorCallback cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_USART_H */