/*
 * Operation.c
 *
 *  Created on: Nov 16, 2023
 *      Author: ahmad
 */
#include "operations.h"
#include <stdlib.h>


struct operation* operation_init(uint8_t op_code, uint8_t length, uint8_t priority, uint8_t number_of_slaves){
	struct operation* temp_operation = (struct operation*)malloc(sizeof(struct operation));
	temp_operation->op_code = op_code;
	temp_operation->priority = priority;
	temp_operation->number_of_working_slaves = number_of_slaves;
	temp_operation->length = length;
	return temp_operation;
}
