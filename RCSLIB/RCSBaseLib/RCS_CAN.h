#ifndef _RCS_CAN1_H_
#define _RCS_CAN1_H_

void RCS_CAN1_Config(uint32_t _baudRate);
void RCS_CAN1_Send(uint32_t Id, uint8_t Length, uint8_t* sendData);
void RCS_CAN1_Recieve(CanRxMsg* RxMessage);

#endif