/*
 * operations_heap.h
 *
 *  Created on: Nov 16, 2023
 *      Author: ahmad
 */

#ifndef SRC_OPHEAP_H_
#define SRC_OPHEAP_H_

#include "../operations.h"
#include <stdint.h>

struct operations_heap{
	struct operation** heap;
	uint8_t heap_size;
	uint8_t number_of_nodes;
	uint8_t is_heap_full;
};

struct operations_heap* operations_heap_init();
void add_opeation_to_heap(struct operations_heap* operations_heap, struct operation* operation);
struct operation* remove_from_operation_heap(struct operations_heap* operations_heap);

#endif /* SRC_OPHEAP_H_ */
