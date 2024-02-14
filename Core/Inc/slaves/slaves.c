/*
 * slaves.c
 *
 *  Created on: Nov 7, 2023
 *      Author: boi
 */

#include "slaves.h"
#include <stdlib.h>
#include <stddef.h>

static struct slaves* idle_slaves_head;
static struct slaves* idle_slaves_tail;
static struct slaves* waiting_slaves_head;
static struct slaves* waiting_slaves_tail;
static struct slaves** busy_slaves;

void slaves_init(){
	idle_slaves_head = NULL;
	idle_slaves_tail = NULL;
	waiting_slaves_head = NULL;
	waiting_slaves_tail = NULL;
	busy_slaves = NULL;
}

void init_busy_slaves_array(uint8_t number_of_slaves){
	busy_slaves = malloc(number_of_slaves * sizeof(struct slaves*));
}

void add_to_idle_slaves(uint8_t slave_id){
	struct slaves* temp_slave = (struct slaves*)malloc(sizeof(struct slaves));
	temp_slave->slave_number = slave_id;
	temp_slave->state = IDLE;
	temp_slave->next = NULL;
	temp_slave->opcode = 0;
	if(idle_slaves_head == NULL){
		idle_slaves_head = temp_slave;
		idle_slaves_tail = temp_slave;
		return;
	}
	idle_slaves_tail->next = temp_slave;
	idle_slaves_tail = temp_slave;
}

struct slaves* get_idle_slaves_list_head(){
	return idle_slaves_head;
}

struct slaves* remove_from_idle_slaves(){
	struct slaves* temp_slave = idle_slaves_head;
	idle_slaves_head = idle_slaves_head->next;
	temp_slave->next = NULL;
	return temp_slave;
}

void add_to_waiting_slaves(uint8_t opcode){
	struct slaves* temp_slave = remove_from_idle_slaves();
	temp_slave->opcode = opcode;
	temp_slave->state = WAITING;
	if(waiting_slaves_head == NULL){
		waiting_slaves_head = temp_slave;
		waiting_slaves_tail = temp_slave;
		return;
	}
	waiting_slaves_tail->next = temp_slave;
	waiting_slaves_tail = temp_slave;
}

struct slaves* remove_from_waiting_slaves(){
	struct slaves* temp_slave = waiting_slaves_head;
	waiting_slaves_head = waiting_slaves_head->next;
	temp_slave->next = NULL;
	return temp_slave;
}

void add_to_busy_slaves(struct slaves* slave, uint8_t index){
	slave->state = BUSY;
	busy_slaves[index] = slave;
}
