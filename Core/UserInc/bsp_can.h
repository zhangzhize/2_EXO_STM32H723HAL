#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"

typedef void (*BspCanRxCallback)(void *ctx, FDCAN_HandleTypeDef *hfdcan, uint32_t can_ext_id, const uint8_t *rx_data);

void BspCanInit(FDCAN_HandleTypeDef *hfdcan);
void FDCanSendData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint32_t id_type, uint8_t *data, uint32_t len);
void BspCanRegisterRxCallback(void *ctx, BspCanRxCallback cb);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */