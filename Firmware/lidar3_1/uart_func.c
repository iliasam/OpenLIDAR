#include "capture_control.h"
#include "speed_finder.h"
#include "stdint.h"
#include "stdio.h"
#include "uart_func.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_dma.h"
#include "string.h"

extern uint16_t res_buf0[PACKET_LENGTH];//result buffer
extern uint16_t res_buf1[PACKET_LENGTH];//result buffer

extern volatile uint8_t overspeed_flag;

volatile extern uint8_t  res_buf_num;//номер буфера, который заполняется в данный момент
//переключаются только при пересечени нуля

extern volatile uint16_t rot_period;//период вращения диска

#define TOTAL_PACKET_LENGTH (uint16_t)(PACKET_LENGTH*2) //packet size in bytes

//младший байт слова передается первым
//определяет, из какого буфера брать данные, и начинает передачу
void start_send_result(void)
{
	/*
	uint16_t i;
	for (i=4;i<364;i++)
	{
		res_buf1[i] = 1500;
		res_buf0[i] = 1500;
	}
	*/

	uint16_t tmp_status_value = 0;
	if (overspeed_flag != 0) tmp_status_value+=(1<<0);//0bit - overspeed flag

	//если сейчас заполняется один буфер, то начинаем передавать другой
	if (res_buf_num == 0)
	{
		res_buf1[0] = 0xBBAA;
		res_buf1[1] = 0xDDCC;
		res_buf1[2] = tmp_status_value;//status flag
		res_buf1[3] = rot_period;//speed
		UART_DMA_start(&res_buf1[0]);
	}
	else
	{
		res_buf0[0] = 0xBBAA;
		res_buf0[1] = 0xDDCC;
		res_buf0[2] = tmp_status_value;//status flag
		res_buf0[3] = rot_period;//speed
		UART_DMA_start(&res_buf0[0]);
	}
}

//configure DMA
void UART_DMA_start(uint16_t *data)
{
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA1_Channel4->CNDTR = TOTAL_PACKET_LENGTH;
	DMA1_Channel4->CMAR = (uint32_t)data;
	//DMA_ClearITPendingBit(DMA1_IT_TC1);
	//DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}


//очиcтить ТЕКУЩИЙ буфер перед началом записи в него
void clear_buffer(void)
{
	//uint16_t i;
	if (res_buf_num == 0)
	{
		memset(&res_buf0[0],0,TOTAL_PACKET_LENGTH);
	}
	else
	{
		memset(&res_buf1[0],0,TOTAL_PACKET_LENGTH);
	}
}


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


void send_test_data(uint16_t value)
{
	uint16_t i;
	uint8_t test_data[16]={32};
	sprintf(&test_data[0],"%d\n",value);
	for (i=0;i<8;i++)
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, (uint16_t)(test_data[i]));
	}
}

