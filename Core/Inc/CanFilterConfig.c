/*
 * CanFilterConfig.c
 *
 *  Created on: Dec 29, 2023
 *      Author: Mohamed Osama
 */



#include "../../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"
#include "CanFilterConfig.h"

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t	Can_RxData[8];
uint8_t	slaves_id[numbers_slaves];
uint8_t	slaves_count = 0;
uint32_t Can_TxMailBox[3];

static ptrVoidFunction CAN_ISR_FUNCTION = NULL;

HAL_StatusTypeDef Can_Filter_Config(CAN_HandleTypeDef *hcan,uint8_t master_or_slave)
{
	if(!master_or_slave)
	{

		CAN_FilterTypeDef	CANFilterConfig;
		CANFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		CANFilterConfig.FilterBank = 10;
		CANFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		CANFilterConfig.FilterIdHigh = can_id<<5;
		CANFilterConfig.FilterIdLow = 0X0000;
		CANFilterConfig.FilterMaskIdHigh = can_id<<5;
		CANFilterConfig.FilterMaskIdLow = 0X0000;
		CANFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		CANFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		CANFilterConfig.SlaveStartFilterBank = 0;
		if( HAL_CAN_ConfigFilter(hcan, &CANFilterConfig) != HAL_OK)
		{
			return HAL_ERROR;
		}
		HAL_CAN_Start(hcan);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

		return HAL_OK;
	}
	else
	{
		CAN_FilterTypeDef	CANFilterConfig;
		CANFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		CANFilterConfig.FilterBank = 10;
		CANFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		CANFilterConfig.FilterIdHigh = CAN_Master_Id<<5;
		CANFilterConfig.FilterIdLow = 0X0000;
		CANFilterConfig.FilterMaskIdHigh = CAN_Master_Id<<5;
		CANFilterConfig.FilterMaskIdLow = 0X0000;
		CANFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		CANFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		CANFilterConfig.SlaveStartFilterBank = 0;

		if( HAL_CAN_ConfigFilter(hcan, &CANFilterConfig) != HAL_OK)
		{
			return HAL_ERROR;
		}
		HAL_CAN_Start(hcan);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
		return HAL_OK;
	}
}

HAL_StatusTypeDef Can_Send(CAN_HandleTypeDef *hcan,uint8_t ID, uint8_t DLC, uint8_t* data,uint32_t* Mailbox)
{
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = DLC;

	if ( HAL_CAN_AddTxMessage(hcan, &TxHeader, data, Mailbox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

void Can_InterruptCallBack(ptrVoidFunction callBackFunction){

	if(callBackFunction != NULL)
		CAN_ISR_FUNCTION = callBackFunction;

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Can_RxData);

	CAN_ISR_FUNCTION();
}

