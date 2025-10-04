#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t *buff;
  size_t size;
  size_t head;
  size_t tail;
} ring_buff_t;

uint8_t ring_buff_init(ring_buff_t *rb, uint8_t *array, size_t array_size);

uint8_t ring_buff_clear(ring_buff_t *rb);

size_t ring_buff_available_raw(size_t buff_size, size_t head, size_t tail);

size_t ring_buff_available(ring_buff_t *rb);

uint8_t ring_buff_put(ring_buff_t *rb, uint8_t *data, size_t data_size);

uint8_t ring_buff_move_head(ring_buff_t *rb, size_t data_size);

size_t ring_buff_get(ring_buff_t *rb, uint8_t *data, size_t data_size);
