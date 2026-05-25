#include "bsp_spi.h"

static void *s_spi_ctx = NULL;
static BspSpiErrorCallback s_spi_err_cb = NULL;

void BspSpiRegisterErrorCallback(void *ctx, BspSpiErrorCallback cb)
{
    s_spi_ctx = ctx;
    s_spi_err_cb = cb;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (s_spi_err_cb != NULL)
    {
        s_spi_err_cb(s_spi_ctx, hspi);
    }
}