/*
 * slaves.h
 *
 *  Created on: Oct 27, 2023
 *      Author: boi
 */

#ifndef SRC_SLAVE_H_
#define SRC_SLAVE_H_

#include <stdint.h>

#define IDLE 	0
#define WAITING 1
#define BUSY 	2

struct slaves{
	uint8_t slave_number;
	uint8_t state;
	uint8_t opcode;
	struct slaves* next;
};

void slaves_init(void);
void init_busy_slaves_array(uint8_t number_of_slaves);
void add_to_idle_slaves(uint8_t slave_id);
struct slaves* get_idle_slaves_list_head(void);
struct slaves* remove_from_idle_slaves(void);
void add_to_waiting_slaves(uint8_t opcode);
struct slaves* remove_from_waiting_slaves(void);
// void add_to_busy_slaves(struct slaves* slave, uint8_t index);

#endif /* SRC_SLAVE_H_ */
