#include "bsp_spi.h"

/* 单例回调上下文：保存 Exo 实例指针 */
static void *s_spi_ctx = NULL;
/* 单例 SPI 错误回调函数指针 */
static BspSpiErrorCallback s_spi_err_cb = NULL;

/**
 * @brief  注册 SPI 错误回调
 * @param  ctx  Exo 对象指针
 * @param  cb   错误回调函数
 */
void BspSpiRegisterErrorCallback(void *ctx, BspSpiErrorCallback cb)
{
    s_spi_ctx = ctx;
    s_spi_err_cb = cb;
}

/**
 * @brief  HAL SPI 发送/接收完成回调 —— 占位
 * @note   BMI088 使用阻塞模式通信，WS2812 的 DMA 完成由 TxCplt 系列回调处理，
 *         此处保留空体供未来扩展（如 DMA 模式的传感器读取）
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{

}

/**
 * @brief  HAL SPI 错误回调 —— 单例模式转发点
 *
 * @details 当 SPI 传输出现错误（CRC、超时、DMA 错误等）时，
 *          HAL 调用此 weak 函数，本模块将其转发到 Exo 实例的错误处理函数。
 *          上层可以根据错误类型决定复位 SPI 外设或重新初始化传感器。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (s_spi_err_cb != NULL)
    {
        s_spi_err_cb(s_spi_ctx, hspi);
    }
}
