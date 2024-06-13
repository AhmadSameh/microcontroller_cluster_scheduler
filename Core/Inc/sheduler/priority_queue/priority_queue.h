/*
 * priority_queue.h
 *
 *  Created on: Jan 19, 2024
 *      Author: ahmad
 */

#ifndef INC_SHEDULER_PRIORITY_QUEUE_PRIORITY_QUEUE_H_
#define INC_SHEDULER_PRIORITY_QUEUE_PRIORITY_QUEUE_H_

#include <stdint.h>

#define MAX_ITEM_NUM	16

typedef struct{
	uint8_t item;
	uint8_t priority;
} queue_item;

typedef struct{
	uint8_t tail;
	queue_item queue[MAX_ITEM_NUM];
} priority_queue;

void priority_queue_init(priority_queue* queue);
void priority_queue_push(priority_queue* queue, uint8_t item, uint8_t priority);
uint8_t priority_queue_pop(priority_queue* queue);
uint8_t priority_queue_peak(priority_queue* queue);

#endif /* INC_SHEDULER_PRIORITY_QUEUE_PRIORITY_QUEUE_H_ */
