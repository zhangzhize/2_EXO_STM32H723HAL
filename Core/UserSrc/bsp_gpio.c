#include "bsp_gpio.h"

/* 单例回调上下文：保存 Exo 实例指针 */
static void *s_gpio_ctx = NULL;
/* 单例外部中断回调函数指针 */
static BspGpioExtiCallback s_gpio_exti_cb = NULL;

/**
 * @brief  注册 GPIO 外部中断回调
 * @param  ctx  Exo 对象指针（用于回调时恢复上下文）
 * @param  cb   中断回调函数
 */
void BspGpioRegisterExtiCallback(void *ctx, BspGpioExtiCallback cb)
{
    s_gpio_ctx = ctx;
    s_gpio_exti_cb = cb;
}

/**
 * @brief  HAL GPIO 外部中断回调 —— 单例模式转发点
 *
 * @details 当任意已使能的 GPIO EXTI 中断触发时，HAL 库调用此 weak 函数。
 *          本模块通过 GPIO_Pin 参数传递触发引脚号，上层 Exo 实例根据
 *          引脚号区分不同的中断源（如急停按钮、限位开关、传感器 DRDY）。
 *
 * @param  GPIO_Pin  触发中断的 GPIO 引脚号（如 GPIO_PIN_0）
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (s_gpio_exti_cb != NULL)
    {
        s_gpio_exti_cb(s_gpio_ctx, GPIO_Pin);
    }
}
