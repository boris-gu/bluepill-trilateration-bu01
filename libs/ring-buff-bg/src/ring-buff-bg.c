#include <string.h>
#include <stdbool.h>
#include "ring-buff-bg.h"

uint8_t ring_buff_init(ring_buff_t *rb, uint8_t *array, size_t array_size) {
  rb->buff = array;
  rb->size = array_size;
  rb->head = 0;
  rb->tail = 0;
  return true;
}


uint8_t ring_buff_clear(ring_buff_t *rb) {
  rb->head = 0;
  rb->tail = 0;
  return true;
}


size_t ring_buff_available_raw(size_t buff_size, size_t head, size_t tail) {
  if (tail <= head) {
    return head - tail;
  }
  return (buff_size - tail) + head;
}


size_t ring_buff_available(ring_buff_t *rb) {
  return ring_buff_available_raw(rb->size, rb->head, rb->tail);
}


uint8_t ring_buff_put(ring_buff_t *rb, uint8_t *data, size_t data_size) {
  size_t local_tail = rb->tail;
  // Проверка на доступное место
  if (rb->size - ring_buff_available_raw(rb->size, rb->head, local_tail) <= data_size) { // = чтобы head не совпал с tail
    return false;
  }

  size_t part1_size = (data_size <= rb->size - rb->head) ? data_size : rb->size - rb->head;
  size_t part2_size = data_size - part1_size;

  memcpy(&rb->buff[rb->head], data, part1_size);
  memcpy(rb->buff, &data[part1_size], part2_size);
  rb->head = (rb->head + data_size) % rb->size;
  return true;
}


uint8_t ring_buff_move_head(ring_buff_t *rb, size_t data_size) {
  size_t local_tail = rb->tail;
  if (rb->size - ring_buff_available_raw(rb->size, rb->head, local_tail) <= data_size) { // = чтобы head не совпал с tail
    return false;
  }
  rb->head = (rb->head + data_size) % rb->size;
  return true;
}


size_t ring_buff_get(ring_buff_t *rb, uint8_t *data, size_t data_size) {
  size_t local_head = rb->head;
  size_t actual_available = ring_buff_available_raw(rb->size, local_head, rb->tail);
  if (actual_available == 0) {
    return 0;
  }

  data_size = data_size < actual_available ? data_size : actual_available;

  size_t part1_size = (data_size <= rb->size - rb->tail) ? data_size : rb->size - rb->tail;
  size_t part2_size = data_size - part1_size;

  memcpy(data, &rb->buff[rb->tail], part1_size);
  memcpy(&data[part1_size], rb->buff, part2_size);
  rb->tail = (rb->tail + data_size) % rb->size;
  return actual_available;
}