#ifndef BSP_USART_H
#define BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#define UART8_RX_BUF_SIZE             256
#define UART9_RX_BUF_SIZE             256
extern uint8_t uart8_rx_buffer[UART8_RX_BUF_SIZE];
extern uint8_t uart9_rx_buffer[UART9_RX_BUF_SIZE];

#ifdef __cplusplus
}
#endif

#endif /* BSP_USART_H */