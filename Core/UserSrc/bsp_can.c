#include "bsp_can.h"

#define BSP_CAN_HANDLE hfdcan1

void BspCanInit(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x0000;
    fdcan_filter.FilterID2 = 0x0000;
    HAL_FDCAN_ConfigFilter(&BSP_CAN_HANDLE, &fdcan_filter);

    fdcan_filter.IdType = FDCAN_EXTENDED_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00000000;
    fdcan_filter.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&BSP_CAN_HANDLE, &fdcan_filter);

	HAL_FDCAN_ConfigGlobalFilter(&BSP_CAN_HANDLE, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&BSP_CAN_HANDLE, FDCAN_CFG_RX_FIFO0, 1);

    HAL_FDCAN_Start(&BSP_CAN_HANDLE);
    HAL_FDCAN_ActivateNotification(&BSP_CAN_HANDLE, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void FDCanSendData(uint32_t id, uint32_t id_type, uint8_t *data, uint32_t len)
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
        pTxHeader.FDFormat = FDCAN_FD_CAN;
        if(len <= 12)      pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
        else if(len <= 16) pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
        else if(len <= 20) pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
        else if(len <= 24) pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
        else if(len <= 32) pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
        else if(len <= 48) pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
        else               pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
    }
    else
    {
        pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
        pTxHeader.DataLength = len;
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&BSP_CAN_HANDLE, &pTxHeader, data);
}

static uint32_t FDCanReceive(FDCAN_HandleTypeDef *hfdcan, uint32_t *rec_id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
    
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		return pRxHeader.DataLength;
	}
	return 0;
}

extern struct Exo *g_exo;
extern void CallExoCanRxCallBack(struct Exo *ptr_exo, FDCAN_HandleTypeDef *hfdcan, uint32_t can_ext_id, uint8_t *rx_data);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint32_t can_id = 0;
    uint8_t rx_data[64] = {0};
    uint32_t rx_len = 0;
    if(hfdcan == &BSP_CAN_HANDLE)
	{   
        rx_len = FDCanReceive(&BSP_CAN_HANDLE, &can_id, rx_data);
        if (rx_len == FDCAN_DLC_BYTES_8)
        {
            CallExoCanRxCallBack(g_exo, hfdcan, can_id, rx_data);
        }
	}
}