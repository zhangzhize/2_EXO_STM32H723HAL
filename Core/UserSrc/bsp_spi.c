#include "spi.h"

extern struct Exo *g_exo;
void CallExoSpiErrorCallback(struct Exo *exo, SPI_HandleTypeDef *hspi);

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    CallExoSpiErrorCallback(g_exo, hspi);
}