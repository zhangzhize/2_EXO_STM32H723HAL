#include "bsp_can.h"

/* 单例回调上下文：保存 Exo 实例指针 */
static void *s_can_ctx = NULL;
/* 单例回调函数指针：由上层注册，中断时调用 */
static BspCanRxCallback s_can_rx_cb = NULL;

/**
 * @brief  注册 CAN 接收回调 —— 单例模式入口
 * @param  ctx  Exo 对象指针（通常为 this）
 * @param  cb   接收回调函数指针（通常指向 Exo 静态成员函数）
 */
void BspCanRegisterRxCallback(void *ctx, BspCanRxCallback cb)
{
  s_can_ctx = ctx;
  s_can_rx_cb = cb;
}

/**
 * @brief  初始化 FDCAN 外设：配置两级过滤器 + 水印中断
 * @param  hfdcan  HAL FDCAN 句柄指针
 *
 * @details 过滤策略：
 *          - 先配置标准 ID 全通（FilterID1/2 = 0x000，掩码模式全匹配）
 *          - 再配置扩展 ID 全通（同理）
 *          - ConfigGlobalFilter 拒绝未命中过滤器的标准/扩展帧及远程帧
 *          - RxFIFO0 水印设为 1，每收到 1 帧即触发中断
 *          - 最后启动外设并激活 RX_FIFO0_NEW_MESSAGE 中断
 */
void BspCanInit(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef fdcan_filter;

  /* 配置标准 ID 过滤器：全接收 */
  fdcan_filter.IdType = FDCAN_STANDARD_ID;
  fdcan_filter.FilterIndex = 0;
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  fdcan_filter.FilterID1 = 0x0000;
  fdcan_filter.FilterID2 = 0x0000;
  HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);

  /* 配置扩展 ID 过滤器：全接收 */
  fdcan_filter.IdType = FDCAN_EXTENDED_ID;
  fdcan_filter.FilterIndex = 1; /* 不能和标准 ID 共用 FilterIndex 0 */
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  fdcan_filter.FilterID1 = 0x00000000;
  fdcan_filter.FilterID2 = 0x00000000;
  HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);

  /* 全局过滤：拒绝所有未命中元素列表的标准帧、扩展帧和远程帧 */
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  /* 水印设为 1：每收到 1 帧即触发 RX FIFO0 中断，逐帧处理 */
  HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);

  HAL_FDCAN_Start(hfdcan);
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
 * @brief  发送 CAN 报文 —— 自动选择帧格式
 * @param  hfdcan  HAL FDCAN 句柄
 * @param  id      报文 ID
 * @param  id_type FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID
 * @param  data    数据缓冲区指针
 * @param  len     数据字节数
 *
 * @details 帧格式选择逻辑：
 *          - len <= 8：使用经典 CAN 帧 (FDCAN_CLASSIC_CAN)
 *          - len > 8：使用 CAN FD 帧，根据实际长度向上取整到 FDCAN DLC 等级
 *            支持的 DLC 等级：12, 16, 20, 24, 32, 48, 64 字节
 */
void FDCanSendData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint32_t id_type, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef pTxHeader;

  pTxHeader.Identifier = id;
  pTxHeader.IdType = id_type;
  pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
  pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  pTxHeader.MessageMarker = 0;
  if (len > 8)
  {
    /* CAN FD 帧：根据数据长度选择最小的 DLC 等级 */
    pTxHeader.FDFormat = FDCAN_FD_CAN;
    if (len <= 12)
      pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
    else if (len <= 16)
      pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
    else if (len <= 20)
      pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
    else if (len <= 24)
      pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
    else if (len <= 32)
      pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
    else if (len <= 48)
      pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
    else
      pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
  }
  else
  {
    /* 经典 CAN 帧：DLC 直接等于数据长度 */
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    pTxHeader.DataLength = len;
  }
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data);
}

/**
 * @brief  从 RX FIFO0 读取报文 —— HAL 中断回调内部辅助函数
 * @param  hfdcan HAL FDCAN 句柄
 * @param  rec_id 输出：接收到的报文 ID
 * @param  buf    输出：数据缓冲区（至少 64 字节）
 * @param  buf_size 输入：缓冲区大小
 * @return 实际接收的数据长度（DLC 编码值），失败返回 0
 */
static void FDCanReceive(FDCAN_HandleTypeDef *hfdcan, uint32_t *rec_id, uint8_t *buf, uint32_t *buf_size)
{
  FDCAN_RxHeaderTypeDef pRxHeader;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
  {
    *rec_id = pRxHeader.Identifier;
    *buf_size = pRxHeader.DataLength;
  }
  else
  {
    *buf_size = 0;
  }
}

/**
 * @brief  HAL FDCAN RX FIFO0 中断回调 —— 关键转发点
 *
 * @details 这是 HAL 库定义的回调函数（weak），由 BSP 层重写。
 *          当 FDCAN RX FIFO0 有新消息时，HAL 中断处理函数会调用此回调。
 *          流程：
 *          1. 从 FIFO0 中提取报文（FDCanReceive）
 *          2. 仅当 DLC == 8（经典 CAN 8 字节帧）且回调已注册时，
 *             将数据通过单例回调转发到上层 Exo 对象
 *          DLC 过滤的原因：外骨骼系统中约定经典 CAN 帧均为 8 字节 DLC
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  uint32_t can_id = 0;
  uint8_t rx_data[64] = {0};
  uint32_t rx_len = 0;

  FDCanReceive(hfdcan, &can_id, rx_data, &rx_len);

  if (s_can_rx_cb != NULL)
  {
    s_can_rx_cb(s_can_ctx, hfdcan, can_id, rx_data, rx_len);
  }
}
