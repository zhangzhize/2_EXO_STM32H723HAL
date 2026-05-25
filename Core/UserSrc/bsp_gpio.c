#include "bsp_gpio.h"

static void *s_gpio_ctx = NULL;
static BspGpioExtiCallback s_gpio_exti_cb = NULL;

void BspGpioRegisterExtiCallback(void *ctx, BspGpioExtiCallback cb)
{
    s_gpio_ctx = ctx;
    s_gpio_exti_cb = cb;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (s_gpio_exti_cb != NULL)
    {
        s_gpio_exti_cb(s_gpio_ctx, GPIO_Pin);
    }
}