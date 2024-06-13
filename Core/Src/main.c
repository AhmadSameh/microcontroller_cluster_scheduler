/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2023 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CanFilterConfig.h"
#include "sheduler/sheduler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ROM_sheduler sheduler;
uint8_t UART2_rxBuffer[8] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void system_callback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void reset_all_select_pins(void){
	for(uint8_t i=0; i<MAX_SLAVE_NUM; i++)
		HAL_GPIO_WritePin(GPIOB, sheduler.slave_pins[i], GPIO_PIN_RESET);
}

void set_current_pin(void){
	HAL_GPIO_WritePin(GPIOB, sheduler.slave_pins[sheduler.selected_pin], GPIO_PIN_SET);
	sheduler.selected_pin++;
}

void add_slave_to_sheduler(void){
	HAL_TIM_Base_Stop_IT(&htim2);
	// deselect any slave
	reset_all_select_pins();
	// add slave to sheduler
	add_idle_slave(&sheduler, RxHeader.StdId, sheduler.number_of_slaves);
	// if the next slave is the last one, change sheduler state
	if(sheduler.number_of_slaves == MAX_SLAVE_NUM || sheduler.selected_pin == MAX_SLAVE_NUM){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		sheduler.selected_pin = -1;
		HAL_TIM_Base_Start_IT(&htim3);
		uint8_t msg[1];
		msg[0] = INTERFACE_READY_SIGNAL;
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, 1, 10);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		return;
	}
	set_current_pin();
	HAL_TIM_Base_Start_IT(&htim2);
}

void send_opcode_to_slave(void){
	// extract operation to be sent from the priority queue and the slave from the slave stack
	sheduler.process_being_sent = sheduler.operation_blocks[priority_queue_peak(&sheduler.operations)];
	uint8_t operation_id = priority_queue_peak(&sheduler.operations);
	uint8_t slave_recieving_number = stack_peak(&sheduler.idle_slaves_stack);
	// if the current operation needs more slaves than available, move on
	if(sheduler.number_of_idle_slaves < sheduler.process_being_sent.number_of_working_slaves){
		sheduler.sheduler_state = IDLE_STATE;
		return;
	}
	// extract operation data to be sent
	uint8_t data[8];
	data[0] = (uint8_t)FRAME_SIGNAL;
	data[1] = sheduler.operation_blocks[operation_id].operation_ID;
	data[2] = (uint8_t)(sheduler.operation_blocks[operation_id].operation_length >> 8);
	data[3] = (uint8_t)(sheduler.operation_blocks[operation_id].operation_length & 0xFF);
	// set state to waiting ack then send to ensure interrupt works
	Can_Send(&hcan, sheduler.slave_blocks[slave_recieving_number].slave_ID, 4, data, &Can_TxMailBox[0]);
	sheduler.sheduler_state = WAITING_ACK_STATE;
	return;
}

void give_rom_to_slave(void){
	give_slave_access_to_ROM(&sheduler);
	// send ROM signal to the slave to be in the ROM
	uint8_t data[8];
	data[0] = ROM_SIGNAL;
	Can_Send(&hcan, sheduler.slave_blocks[sheduler.current_slave_in_ROM].slave_ID, 1, data, &Can_TxMailBox[0]);
}

void add_slave_to_waiting_queue(void){
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	uint8_t operation_id = priority_queue_peak(&sheduler.operations);
	uint8_t slave_recieving_number = stack_pop(&sheduler.idle_slaves_stack);
	// give the slave the current operation
	give_slave_opcode(&sheduler, sheduler.operation_blocks[operation_id], slave_recieving_number);
	// change state to sending code again if there are still operations to send and slaves to receive
	// if slave received the opcode, the slave sends a RCV_ACK
	if(sheduler.is_ROM_available){
		// move the slave from the waiting queue
		give_rom_to_slave();
	}
	if(sheduler.number_of_idle_slaves != 0 && sheduler.number_of_available_operations != 0)
		sheduler.sheduler_state = SENDING_CODE_STATE;
	else
		sheduler.sheduler_state = IDLE_STATE;
	return;
}

void set_slave_busy(void){
	// set rom is available
	sheduler.is_ROM_available = 1;
	// change the slave's state to idle
	sheduler.slave_blocks[sheduler.current_slave_in_ROM].slave_state = SLAVE_WORKING;
	// if there are more waiting slaves, give one access to ROM
	if(sheduler.number_of_waiting_slaves != 0)
		give_rom_to_slave();
}

void select_reset_slave(uint32_t id){
	for(uint8_t i=0; i<sheduler.number_of_slaves; i++){
		if(sheduler.slave_blocks[i].slave_ID == id){
			sheduler.selected_pin = sheduler.slave_blocks[i].pin;
			break;
		}
	}
	reset_all_select_pins();
	set_current_pin();
}

void send_status_to_interface(void){
	for(uint8_t i=0; i<MAX_SLAVE_NUM; i++){
		if(sheduler.slave_blocks[i].slave_ID == -1){
			continue;
		}
		uint8_t msg[5];
		msg[0] = INTERFACE_STATUS_MSG;
		msg[1] = sheduler.slave_blocks[i].slave_number;
		msg[2] = sheduler.slave_blocks[i].slave_state;
		msg[3] = sheduler.slave_blocks[i].current_opcode;
		if(i != MAX_SLAVE_NUM - 1){
			msg[4] = INTERFACE_MORE_SIGNAL;
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, 5, 10);
			continue;
		}
		msg[4] = INTERFACE_DONE_SIGNAL;
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, 5, 10);
	}
}

void send_slave_ids_to_interface(void){
	for(uint8_t i=0; i<MAX_SLAVE_NUM; i++){
		if(sheduler.slave_blocks[i].slave_ID == -1){
			continue;
		}
		uint8_t msg[10];
		msg[0] = INTERFACE_ID_MSG;
		msg[1] = sheduler.slave_blocks[i].slave_ID;
		msg[2] = sheduler.slave_blocks[i].slave_number;
		msg[3] = 0;
		msg[4] = 0;
		msg[5] = 0;
		msg[6] = 0;
		msg[7] = 0;
		msg[8] = 0;
		if(i < MAX_SLAVE_NUM - 1) {
			msg[9] = INTERFACE_MORE_SIGNAL;
		}
		else {
			msg[9] = INTERFACE_DONE_SIGNAL;
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, 10, 10);
		while(1) {
			HAL_UART_Receive(&huart2, UART2_rxBuffer, 8, 10);
			if(UART2_rxBuffer[0] == INTERFACE_ACK) {
				break;
			}
		}
	}
}

void kill_slave_process(uint8_t slave_number){
	uint8_t data[8];
	data[0] = RESET_SIGNAL;
	Can_Send(&hcan, sheduler.slave_blocks[slave_number].slave_ID, 1, data, &Can_TxMailBox[0]);
	set_slave_idle(&sheduler, slave_number);
}

void run_operation(uint8_t slave_id, uint8_t index, uint8_t length_high, uint8_t length_low, uint8_t priority, uint8_t number_of_slaves){
	// if slave is selected by id, run it immediately
	if(slave_id != MAX_SLAVE_NUM){
		uint8_t data[8];
		data[0] = (uint8_t)FRAME_SIGNAL;
		data[1] = index;
		data[2] = length_high;
		data[3] = length_low;
		Can_Send(&hcan, sheduler.slave_blocks[slave_id-1].slave_ID, 4, data, &Can_TxMailBox[0]);
		sheduler.sheduler_state = WAITING_ACK_STATE;
		return;
	}
	operation_control_block operation;
	operation.operation_ID = index;
	uint16_t length = 0;
	length = (length_high << 8) | length_low;
	operation.operation_length = length;
	operation.number_of_working_slaves = number_of_slaves;
	operation.operation_priority = priority;
	add_operation(&sheduler, operation);
}

void alert_slave_finished(uint8_t slave_number){
	uint8_t msg[10];
	msg[0] = INTERFACE_SLAVE_DONE;
	msg[1] = slave_number;
	msg[2] = 0;
	msg[3] = 0;
	msg[4] = 0;
	msg[5] = 0;
	msg[6] = 0;
	msg[7] = 0;
	msg[8] = 0;
	msg[9] = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, 10, 10);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Can_Filter_Config(&hcan, master);
//   initiate can receive callback
  Can_InterruptCallBack(system_callback);
//   select pins for initialization
  sheduler_init(&sheduler);
//   create 2 dummy operations then add it to the operation priority queue
//   this approach is used since the interface microcontroller is not ready
  operation_control_block operation;
  operation.operation_ID = 0x00;
  operation.operation_length = 0x1900;
  operation.number_of_working_slaves = 1;
  operation.operation_priority = 1;
  add_operation(&sheduler, operation);
  add_operation(&sheduler, operation);
  add_operation(&sheduler, operation);
  add_operation(&sheduler, operation);
//   select the first slave
  reset_all_select_pins();
  set_current_pin();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 8);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * Scheduler <--> Slave Communications
 **/
void system_callback(){
	// INIT state adds slaves to the slave control block
	if(sheduler.sheduler_state == INIT_STATE){
		add_slave_to_sheduler();
		return;
	}
	// if the reset slave sends ack it was reset
	if(Can_RxData[0] == RESET_SIGNAL){
		set_slave_idle_by_id(&sheduler, RxHeader.StdId);
		return;
	}
	// send opcode to any slave
	if(sheduler.sheduler_state == SENDING_CODE_STATE){
		send_opcode_to_slave();
		return;
	}
	// received ACK and now adding the slave to the waiting queue
	if(sheduler.sheduler_state == WAITING_ACK_STATE && Can_RxData[0] == RCVD_ACK){
		add_slave_to_waiting_queue();
		return;
	}
	// if slave is finished accessing the ROM
	if(Can_RxData[0] == MEM_FREE_ACK){
		set_slave_busy();
		return;
	}
	// if slave finished working on operation
	if(Can_RxData[0] == WORK_DONE_ACK){
		set_slave_idle(&sheduler, Can_RxData[1]);
		alert_slave_finished(RxHeader.StdId);
		return;
	}
}

/*
 * Scheduler <--> Interface Communications
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(UART2_rxBuffer[0] == INTERFACE_START_SIGNAL){
		sheduler.sheduler_state = IDLE_STATE;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		send_slave_ids_to_interface();
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		return;
	}
	if(UART2_rxBuffer[0] == INTERFACE_STATUS_SIGNAL){
		send_status_to_interface();
		return;
	}
	if(UART2_rxBuffer[0] == INTERFACE_RUN_SIGNAL){
		run_operation(UART2_rxBuffer[1], UART2_rxBuffer[2], UART2_rxBuffer[3], UART2_rxBuffer[4], UART2_rxBuffer[5], UART2_rxBuffer[6]);
		return;
	}
	if(UART2_rxBuffer[0] == INTERFACE_KILL_SIGNAL){
		kill_slave_process(UART2_rxBuffer[1]);
		return;
	}
	if(UART2_rxBuffer[0] == INTERFACE_ROM_SIGNAL) {
		uint8_t data[1] = {0};
		if(sheduler.is_ROM_available && sheduler.number_of_waiting_slaves == 0) {
			data[0] = 1;
		}
		HAL_UART_Transmit(huart, data, 1, 10);
	}
	HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 8);
}

/*
 * Scheduler tick time
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM2){
    	if(sheduler.sheduler_state == INIT_STATE) {
    		reset_all_select_pins();
    		set_current_pin();
    		if(sheduler.selected_pin >= MAX_SLAVE_NUM) {
    			uint8_t msg[10];
    			msg[0] = INTERFACE_READY_SIGNAL;
    			msg[1] = 0;
    			msg[2] = 0;
    			msg[3] = 0;
    			msg[4] = 0;
    			msg[5] = 0;
    			msg[6] = 0;
    			msg[7] = 0;
    			msg[8] = 0;
    			msg[9] = 0;
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, 10, 10);
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			}
    		return;
    	}
    	return;
    }
    if(htim->Instance == TIM3){
    	if(sheduler.sheduler_state == IDLE_STATE && sheduler.number_of_idle_slaves != 0 && sheduler.number_of_available_operations != 0){
    		sheduler.sheduler_state = SENDING_CODE_STATE;
    		send_opcode_to_slave();
			return;
    	}
    	return;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
