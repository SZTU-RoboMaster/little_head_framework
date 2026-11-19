//
// Created by hjr on 2026/11/11.
//

#ifndef G4_LITTLE_HEAD_BOARD_COMMUNICATION_H
#define G4_LITTLE_HEAD_BOARD_COMMUNICATION_H
#include "bsp_can.h"

void Boardcomm_Gimbal_Send(void);
void Boardcomm_Gimbal_Receive(BspCan_RxBuffer_t *rx_buffer);
void Boardcomm_Chassis_Send(void);
void Boardcomm_Chssais_Receive(BspCan_RxBuffer_t *rx_buffer);

#endif // G4_LITTLE_HEAD_BOARD_COMMUNICATION_H


