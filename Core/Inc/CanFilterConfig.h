/*
 * CanFilterConfig.h
 *
 *  Created on: Dec 29, 2023
 *      Author: Mohamed Osama
 */



#include "../../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"


#ifndef CANFILTERCONFIG_H_
#define CANFILTERCONFIG_H_

#define numbers_slaves	4
#define master			1
#define slave			0
#define U_Id_Address  	(uint32_t*)0X1FFFF7E8
#define CAN_Master_Id	0x0000
#define can_id			(*(U_Id_Address)^*(U_Id_Address + 0X1)^*(U_Id_Address + 0X2))>>24

typedef void (*ptrVoidFunction)(void);

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t Can_TxMailBox[3];
extern uint8_t	Can_RxData[8];
extern uint8_t	slaves_id[numbers_slaves];
extern uint8_t	slaves_count;



HAL_StatusTypeDef Can_Filter_Config(CAN_HandleTypeDef *hcan,uint8_t master_or_slave);
HAL_StatusTypeDef Can_Send(CAN_HandleTypeDef *hcan,uint8_t ID, uint8_t DLC, uint8_t* data,uint32_t* Mailbox);
void Can_InterruptCallBack(ptrVoidFunction callBackFunction);



#endif /* CANFILTERCONFIG_H_ */
