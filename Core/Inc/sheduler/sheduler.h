/*
 * sheduler.h
 *
 *  Created on: Oct 27, 2023
 *      Author: boi
 */

#ifndef SRC_SHEDULER_H_
#define SRC_SHEDULER_H_

// #include "slaves/slaves.h"
// #include "operations/operations.h"
// #include "operations/operations_heap/operations_heap.h"
#include "stack/stack.h"
#include "priority_queue/priority_queue.h"

#define RESET_ACK 		0x01
#define RCVD_ACK 		0x02
#define MEM_FREE_ACK	0x03
#define WORK_DONE_ACK	0x04

#define INIT_STATE			0x01
#define IDLE_STATE			0x02
#define SENDING_CODE_STATE	0x03
#define WAITING_ACK_STATE	0x04
#define WAITING_SIG_STATE	0x05

#define SLAVE_IDLE		0x00
#define SLAVE_WAITING	0x01
#define SLAVE_IN_ROM 	0x02
#define SLAVE_WORKING 	0x03

#define START_SIGNAL	0x01
#define FRAME_SIGNAL	0x30
#define ROM_SIGNAL 		0x40
#define RESET_SIGNAL	0x50

// sent by scheduler
#define INTERFACE_READY_SIGNAL	0x41
#define INTERFACE_STATUS_MSG	0x43
#define INTERFACE_MORE_SIGNAL	0x45
#define INTERFACE_DONE_SIGNAL 	0x47
#define INTERFACE_ID_MSG		0x50
#define INTERFACE_ACK 			0x51
// sent by interface
#define INTERFACE_START_SIGNAL 		0x40
#define INTERFACE_REQUEST_SIGNAL	0x42
#define INTERFACE_STATUS_SIGNAL		0x44
#define INTERFACE_RUN_SIGNAL		0x46
#define INTERFACE_KILL_SIGNAL		0x48
#define INTERFACE_ROM_SIGNAL 		0x49

#define MAX_SLAVE_NUM		4
#define MAX_OPERATIONS_NUM	64

#define SCHEDULE_ON_PRIORITY 0x01
#define SCHEDULE_ON_NUMBERS 0x02
#define SCHEDULING_TECHNIQUE SCHEDULE_ON_PRIORITY

typedef struct{
	int8_t slave_ID;
	uint8_t slave_state;
	uint8_t slave_number;
	uint8_t current_opcode;
	uint16_t pin;
} slave_control_block;

typedef struct{
	uint8_t operation_ID;
	uint8_t operation_priority;
	uint8_t number_of_working_slaves;
	uint16_t operation_length;
} operation_control_block;

typedef struct{
	// struct slaves* current_slave_in_ROM;
	slave_control_block slave_blocks[MAX_SLAVE_NUM];
	operation_control_block operation_blocks[MAX_OPERATIONS_NUM];
	stack idle_slaves_stack;
	priority_queue waiting_slaves_queue;
	priority_queue operations;
	uint8_t current_slave_in_ROM;
	uint8_t is_ROM_available;
	uint8_t number_of_slaves;
	uint8_t number_of_idle_slaves;
	uint8_t number_of_waiting_slaves;
	uint8_t number_of_working_slaves;
	uint8_t	number_of_available_operations;
	uint8_t sheduler_state;
	uint16_t slave_pins[MAX_SLAVE_NUM];
	int16_t selected_pin;
	operation_control_block process_being_sent;
} ROM_sheduler;

void sheduler_init(ROM_sheduler* sheduler);
void add_idle_slave(ROM_sheduler* sheduler, uint8_t slave_id, uint8_t slave_num);
void add_operation(ROM_sheduler* sheduler, operation_control_block new_operation);
void give_slave_opcode(ROM_sheduler* sheduler, operation_control_block slave_operation, uint8_t slave_recieving_number);
void give_slave_access_to_ROM(ROM_sheduler* sheduler);
void set_slave_idle(ROM_sheduler* sheduler, uint8_t slave_num);
void set_slave_idle_by_id(ROM_sheduler* sheduler, uint32_t id);

#endif /* SRC_SHEDULER_H_ */
