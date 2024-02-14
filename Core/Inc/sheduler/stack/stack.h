/*
 * stack.h
 *
 *  Created on: Jan 19, 2024
 *      Author: ahmad
 */

#ifndef INC_SHEDULER_STACK_STACK_H_
#define INC_SHEDULER_STACK_STACK_H_

#include <stdint.h>

#define MAX_ITEM_NUM	16

typedef struct{
	uint8_t top;
	uint8_t item[MAX_ITEM_NUM];
} stack;

void stack_init(stack* _stack);
void stack_push(stack* _stack, uint8_t item);
uint8_t stack_pop(stack* _stack);
uint8_t stack_peak(stack* _stack);

#endif /* INC_SHEDULER_STACK_STACK_H_ */
