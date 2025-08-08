#ifndef _MY_CAN_H
#define _MY_CAN_H


#include "main.h"
#include "libelybot_can.h"

extern FDCAN_RxHeaderTypeDef fdcan_rx_header1;
extern uint8_t fdcan1_rdata[24];
extern motor_state_t motor_state;
extern uint8_t motor_read_flag;


uint8_t can_send(FDCAN_HandleTypeDef *hfdcanx, uint32_t id, uint8_t *msg, uint8_t len);
void fdcan_filter_init(FDCAN_HandleTypeDef *fdcanHandle);
uint8_t Fdcan_Dlc_To_Len(uint32_t dlc);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);


//uint32_t get_fdcan_dlc(uint16_t size);
//uint16_t get_fdcan_data_size(uint32_t dlc);
//void fdcan_filter_init(FDCAN_HandleTypeDef *fdcanHandle);
//void fdcan_send(FDCAN_HandleTypeDef *fdcanHandle, uint32_t id, uint8_t *data, uint16_t size);



#endif
