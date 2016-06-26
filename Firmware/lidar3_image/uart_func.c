#include "stdint.h"
#include "uart_func.h"
#include "stm32f30x_usart.h"

void uart_send_array(uint16_t *array, uint16_t length)
{
	uint16_t i;

	for (i=0;i<length;i++)
	{
	    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	    USART_SendData(USART1, (uint16_t)(array[i]>>8));
	    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	    USART_SendData(USART1, (uint16_t)(array[i] & 0xff));
	}
}
