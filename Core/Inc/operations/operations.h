/*
 * Operation.h
 *
 *  Created on: Nov 16, 2023
 *      Author: ahmad
 */

#ifndef SRC_OPERATION_H_
#define SRC_OPERATION_H_

#include <stdint.h>

struct operation{
	uint8_t op_code;
	uint8_t priority;
	uint16_t length;
	uint8_t number_of_working_slaves;
};

struct operation* operation_init(uint8_t op_code, uint8_t length, uint8_t priority, uint8_t number_of_slaves);

#endif /* SRC_OPERATION_H_ */
