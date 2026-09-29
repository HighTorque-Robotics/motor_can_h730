#ifndef _MY_CAN_H
#define _MY_CAN_H


#include "main.h"

uint32_t can_size2dlc(uint16_t size);
uint16_t can_dlc2size(uint32_t dlc);

void can_filter_init(FDCAN_HandleTypeDef *fdcanHandle);
void can_send(FDCAN_HandleTypeDef *hfdcanx, uint32_t id, uint8_t *data, uint8_t len);






#endif
