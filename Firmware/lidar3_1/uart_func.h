#ifndef _UART_FUNC
#define _UART_FUNC
#include "stdint.h"

void uart_send_array(uint16_t *array, uint16_t length);
void UART_DMA_start(uint16_t *data);
void start_send_result(void);
void clear_buffer(void);
void send_test_data(uint16_t value);

#endif
