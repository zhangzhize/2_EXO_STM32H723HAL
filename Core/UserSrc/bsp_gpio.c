#include "bsp_gpio.h"

extern struct Exo *g_exo;
void CallExoSpiRxStart(struct Exo *exo);
void CallExoSpiRxCallback(struct Exo *exo);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case NRF54_CS_INT_Pin:
        if (HAL_GPIO_ReadPin(NRF54_CS_INT_GPIO_Port, NRF54_CS_INT_Pin) == GPIO_PIN_RESET)
        {
            CallExoSpiRxStart(g_exo);
        }
        else if (HAL_GPIO_ReadPin(NRF54_CS_INT_GPIO_Port, NRF54_CS_INT_Pin) == GPIO_PIN_SET)
        {
            CallExoSpiRxCallback(g_exo);
        }
        break;
        
    default:
        break;
    }
}