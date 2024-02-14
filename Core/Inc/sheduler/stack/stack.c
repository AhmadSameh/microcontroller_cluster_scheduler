/*
 * stack.c
 *
 *  Created on: Jan 19, 2024
 *      Author: ahmad
 */

#include "stack.h"

void stack_init(stack* _stack){
	_stack->top = 0;
}

void stack_push(stack* _stack, uint8_t item){
	// TODO: add here an error state
	if(_stack->top >= MAX_ITEM_NUM)
		return;
	// add item to the top of stack, top = number of elements
	_stack->item[_stack->top] = item;
	_stack->top++;
}

uint8_t stack_pop(stack* _stack){
	// TODO: add here an error state
	if(_stack->top <= 0)
			return 0;
	// decrement top index, top becomes the last element
	_stack->top--;
	return _stack->item[_stack->top];
}

uint8_t stack_peak(stack* _stack){
	return _stack->item[_stack->top-1];
}
