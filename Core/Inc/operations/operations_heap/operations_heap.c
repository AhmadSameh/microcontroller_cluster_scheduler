/*
 * OpHeap.c
 *
 *  Created on: Nov 16, 2023
 *      Author: ahmad
 */


#include "operations_heap.h"
#include "../operations.h"
#include <stdint.h>
#include <stdlib.h>

static void swap_operations(struct operation* operation1, struct operation* operation2);
static void heapify(struct operations_heap* op_heap, uint8_t index);

struct operations_heap* operations_heap_init(){
	struct operations_heap* temp_heap = (struct operations_heap*)malloc(sizeof(struct operations_heap));
	temp_heap->heap_size = 0;
	temp_heap->is_heap_full = 0;
	temp_heap->number_of_nodes = 0;
	return temp_heap;
}

void add_opeation_to_heap(struct operations_heap* op_heap, struct operation* opcode){
	if(op_heap->heap_size == 0){
		op_heap->heap_size++;
		op_heap->heap = malloc(sizeof(struct operation*) * op_heap->heap_size);
	}else if(op_heap->is_heap_full){
		op_heap->heap_size++;
		struct operation** temp_heap = malloc(sizeof(struct operation*) * op_heap->heap_size);
		struct operation** temp_heap2 = op_heap->heap;
		for(uint8_t i=0; i<op_heap->number_of_nodes; i++){
			struct operation* temp_operation = operation_init(op_heap->heap[i]->op_code, op_heap->heap[i]->length, op_heap->heap[i]->priority, op_heap->heap[i]->number_of_working_slaves);
			temp_heap[i] = temp_operation;
		}
		op_heap->heap = temp_heap;
		free(temp_heap2);
	}
	op_heap->heap[op_heap->number_of_nodes] = opcode;
	op_heap->number_of_nodes++;
	if(op_heap->number_of_nodes == op_heap->heap_size)
		op_heap->is_heap_full = 1;

	for(int i=(op_heap->number_of_nodes/2)-1; i>=0; i--)
		heapify(op_heap, i);
}

struct operation* remove_from_operation_heap(struct operations_heap* op_heap){
	swap_operations(op_heap->heap[0], op_heap->heap[op_heap->number_of_nodes - 1]);
	op_heap->number_of_nodes--;
	heapify(op_heap, 0);
	return op_heap->heap[op_heap->number_of_nodes];
}

static void heapify(struct operations_heap* op_heap, uint8_t index){
	uint8_t smallest = index;
	struct operation* parent = op_heap->heap[index];
	uint8_t left_index = index * 2 + 1;
	struct operation* left_child = op_heap->heap[left_index];
	uint8_t right_index = left_index + 1;
	struct operation* right_child = op_heap->heap[right_index];

	if(left_index < op_heap->number_of_nodes && left_child->priority < parent->priority)
		smallest = left_index;

	if(right_index < op_heap->number_of_nodes && right_child->priority < parent->priority)
			smallest = right_index;

	if(smallest != index){
		swap_operations(op_heap->heap[smallest], op_heap->heap[index]);
		heapify(op_heap, smallest);
	}
}

static void swap_operations(struct operation* operation1, struct operation* operation2){
	struct operation temp_slave = *operation1;
	*operation1 = *operation2;
	*operation2 = temp_slave;
}
