/*
 * priority_queue.c
 *
 *  Created on: Jan 19, 2024
 *      Author: ahmad
 */

#include "priority_queue.h"
#include <stdint.h>

void heapify(priority_queue* queue, uint8_t index);
void swap_operations(priority_queue* queue, uint8_t index1, uint8_t index2);

void priority_queue_init(priority_queue* queue){
	queue->tail = 0;
}

void priority_queue_push(priority_queue* queue, uint8_t item, uint8_t priority){
	// TODO: add here an error state
	if(queue->tail >= MAX_ITEM_NUM)
		return;
	// add queue_item to the queue at index tail [x x x tail] <----- three elements, tail = 3, tail = number of elements
	queue->queue[queue->tail].item = item;
	queue->queue[queue->tail].priority = priority;
	// heapify the queue
	for(int i=(queue->tail-1)/2; i>=0; i--)
		heapify(queue, i);
	// increase number of elements/tail
	queue->tail++;
}

uint8_t priority_queue_pop(priority_queue* queue){
	// TODO: add here an errors state
	if(queue->tail <=0)
		return 0;
	// swap first and last elements
	swap_operations(queue, 0, queue->tail-1);
	// decrement tail so now, last element is out of bounds of the queue
	queue->tail--;
	// heapify the queue
	heapify(queue, 0);
	// return the item of the element of the queue, which is discarded
	return queue->queue[queue->tail].item;
}

uint8_t priority_queue_peak(priority_queue* queue){
	return queue->queue[0].item;
}

void heapify(priority_queue* queue, uint8_t index){
	// store current node as the smallest and save left and right children
	uint8_t smallest = index;
	uint8_t left_child = index * 2 + 1;
	uint8_t right_child = left_child + 1;
	// if left child is smaller than current, left child is the smallest
	if(left_child < queue->tail && queue->queue[left_child].priority < queue->queue[index].priority)
		smallest = left_child;
	// if right child is smaller than current, right child is the smallest
	if(right_child < queue->tail && queue->queue[right_child].priority < queue->queue[index].priority)
			smallest = right_child;
	// if the smallest is not the current, swap smallest with current then heapify again
	if(smallest != index){
		swap_operations(queue, smallest, index);
		heapify(queue, smallest);
	}
}

void swap_operations(priority_queue* queue, uint8_t index1, uint8_t index2){
	queue_item temp = queue->queue[index1];
	queue->queue[index1] = queue->queue[index2];
	queue->queue[index2] = temp;
}
