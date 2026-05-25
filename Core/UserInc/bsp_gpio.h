#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"

typedef void (*BspGpioExtiCallback)(void *ctx, uint16_t GPIO_Pin);

void BspGpioRegisterExtiCallback(void *ctx, BspGpioExtiCallback cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */