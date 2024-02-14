/*
 * sheduler.c
 *
 *  Created on: Oct 27, 2023
 *      Author: boi
 */

#include "sheduler.h"
#include <stdlib.h>

void sheduler_init(ROM_sheduler* sheduler, uint16_t pins){
	sheduler->current_slave_in_ROM = 0;
	sheduler->number_of_slaves = 0;
	sheduler->number_of_available_operations = 0;
	sheduler->number_of_idle_slaves = 0;
	sheduler->number_of_waiting_slaves = 0;
	sheduler->number_of_working_slaves = 0;
	sheduler->sheduler_state = INIT_STATE;
	sheduler->is_ROM_available = 1;
	sheduler->process_being_sent.operation_ID = 0x00;
	sheduler->selected_pin = -1;
	for(uint8_t i=0; i<MAX_SLAVE_NUM; i++)
		sheduler->slave_pins[i] = pins[i];
	priority_queue_init(&sheduler->operations);
	priority_queue_init(&sheduler->waiting_slaves_queue);
	stack_init(&sheduler->idle_slaves_stack);
}

void sheduler_select_slave(ROM_sheduler* sheduler, uint8_t port, uint16_t pin_number){
	if(sheduler->selected_pin == MAX_SLAVE_NUM){
		sheduler->selected_pin = -1;
		sheduler->sheduler_state = SENDING_CODE_STATE;
	}
	sheduler->selected_pin++;
	for(uint8_t i=0; i<MAX_SLAVE_NUM; i++)
		HAL_GPIO_WritePin(GPIOB, sheduler->slave_pins[i].pins, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, sheduler->slave_pins[sheduler->selected_pin], GPIO_PIN_SET);
}

void add_idle_slave(ROM_sheduler* sheduler, uint8_t slave_id, uint8_t slave_num){
	// TODO: add here an error state
	if(slave_num >= MAX_SLAVE_NUM)
		return;
	// increase number of saved slaves
	sheduler->number_of_slaves++;
	// increase number of known idle slaves
	sheduler->number_of_idle_slaves++;
	// register slave to SCB
	sheduler->slave_blocks[slave_num].slave_ID = slave_id;
	sheduler->slave_blocks[slave_num].slave_number = slave_num;
	sheduler->slave_blocks[slave_num].slave_state = SLAVE_IDLE;
	sheduler->slave_blocks[slave_num].current_opcode = 0x00;
	// push slave number to free slave stack
	stack_push(&sheduler->idle_slaves_stack, slave_num);
}

// TODO: implement send_number_of_idle_slaves(uint8_t num)

void add_operation(ROM_sheduler* sheduler, operation_control_block new_operation){
	// TODO: add here an error state
	if(sheduler->number_of_available_operations >= MAX_OPERATIONS_NUM)
		return;
	// register operation to OCB
	sheduler->operation_blocks[sheduler->number_of_available_operations] = new_operation;
	sheduler->operation_blocks[sheduler->number_of_available_operations].number_of_working_slaves = 0x00;
	// push operation number to operation queue
	priority_queue_push(&sheduler->operations, sheduler->number_of_available_operations, new_operation.operation_priority);
	// increase number of available operations
	sheduler->number_of_available_operations++;
}

// remove slave from the idle stack then push it to the waiting queue with the opcode to be sent
void give_slave_opcode(ROM_sheduler* sheduler, operation_control_block slave_operation, uint8_t slave_recieving_number){
	// TODO: add here an error state
	if(sheduler->number_of_idle_slaves <= 0)
		return;
	// change slave state in SCB
#if SCHEDULING_TECHNIQUE == SCHEDULE_ON_NUMBERS

#else
	for(int i=0; i<slave_operation.number_of_working_slaves; i++){

	}
	sheduler->slave_blocks[slave_recieving_number].slave_state = SLAVE_WAITING;
	// assign slave the operation code in its SCB
	sheduler->slave_blocks[slave_recieving_number].current_opcode = slave_operation.operation_ID;
	// push slave number to waiting slaves queue and use operation priority
	priority_queue_push(&sheduler->waiting_slaves_queue, slave_recieving_number, slave_operation.operation_priority);
	// increase number of waiting slaves
	sheduler->number_of_waiting_slaves++;
	// reduce idle slaves number
	sheduler->number_of_idle_slaves--;
	// decrement number of slaves left
	sheduler->process_being_sent.number_of_working_slaves--;
	// if the operation was sent to the required slaves
	if(sheduler->process_being_sent.number_of_working_slaves == 0){
		// set process to be sent to 0 again
		sheduler->process_being_sent.operation_ID = 0x00;
		// decrement number of available operations
		sheduler->number_of_available_operations--;
		// remove the operation from queue
		priority_queue_pop(&sheduler->operations);
		// if we can still send operations, set process being set
		if(sheduler->number_of_idle_slaves != 0 && sheduler->number_of_available_operations != 0)
			sheduler->process_being_sent = sheduler->operation_blocks[priority_queue_peak(&sheduler->operations)];
	}
#endif
}

// void add_waiting_slave(ROM_sheduler* sheduler, uint8_t acknowledged_opcode){
// 	sheduler->number_of_idle_slaves--;
// 	add_to_waiting_slaves(acknowledged_opcode);
// }

void give_slave_access_to_ROM(ROM_sheduler* sheduler){
	// pop slave from its waiting queue and set the current_slave_in_ROM to it
	sheduler->current_slave_in_ROM = priority_queue_pop(&sheduler->waiting_slaves_queue);
	// decremnt number of waiting slaves
	sheduler->number_of_waiting_slaves--;
	// change the slave's state in the block
	sheduler->slave_blocks[sheduler->current_slave_in_ROM].slave_state = SLAVE_IN_ROM;
	// set ROM is not available
	sheduler->is_ROM_available = 0;
}

// void add_busy_slave(ROM_sheduler* sheduler, struct slaves* current_slave, uint8_t number_of_working_slaves){
// 	// add_to_busy_slaves(current_slave, number_of_working_slaves);
// 	sheduler->number_of_working_slaves++;
// 	sheduler->is_ROM_available = 1;
// 	sheduler->current_slave_in_ROM = 0;
// }
