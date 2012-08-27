#ifndef QUEUE_H
#define QUEUE_H

uint8_t queue_have_space(uint8_t *p_pointer, uint8_t *p_behind, uint8_t queue_size);
uint8_t queue_have_item(uint8_t *p_pointer, uint8_t *p_ahead, uint8_t queue_size);

uint8_t queue_push(uint8_t *p_pointer, uint8_t *p_behind, uint8_t queue_size);
uint8_t queue_pop(uint8_t *p_pointer, uint8_t *p_ahead, uint8_t queue_size);

uint8_t queue_current(uint8_t *p_pointer);

#endif
