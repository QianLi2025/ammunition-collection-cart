#ifndef BSP_CAN_H_
#define BSP_CAN_H_
#include <stdint.h> 
#include "can.h"
#include "M3508.h"

void CAN_Transmit_lower(int16_t m1,int16_t m2,int16_t m3,int16_t m4);
//void CAN_Transmit_upper(int16_t m1,int16_t m2,int16_t m3,int16_t m4);
void CANAddFilter();
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox);

#endif // BSP_CAN_H_