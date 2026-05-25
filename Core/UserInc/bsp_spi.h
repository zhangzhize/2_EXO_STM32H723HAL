#ifndef BSP_SPI_H
#define BSP_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

typedef void (*BspSpiErrorCallback)(void *ctx, SPI_HandleTypeDef *hspi);

void BspSpiRegisterErrorCallback(void *ctx, BspSpiErrorCallback cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */