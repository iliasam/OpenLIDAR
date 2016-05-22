#include "hardware.h"
#include "uart_func.h"
#include "adc_control.h"
#include "capture_control.h"
#include "speed_finder.h"

#include "stm32f30x_rcc.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_comp.h"

//#define ADJ_MODE

extern volatile uint16_t line_buffer0[LINE_BUFFER_SIZE];
extern volatile uint16_t line_buffer1[LINE_BUFFER_SIZE];
extern volatile cap_status_type image_cap_status;
extern volatile uint16_t cap_number;//число захваченных точек

RCC_ClocksTypeDef RCC_Clocks;

uint16_t header[3] = {0xAABB,0xCCDD,0xEEFF};

volatile uint16_t tmp_range = 0;

int main(void)
{
	init_hardware();

    RCC_GetClocksFreq (&RCC_Clocks);
    set_led_on();

    //SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

    Delay_ms(500);
    set_led_off();
    Delay_ms(100);

    refresh_sync_timer(40);//for testing

#ifndef ADJ_MODE
    while(1)
    {
    	scan_capture_handler();
    }
#endif

#ifdef ADJ_MODE
    TIM_Cmd(TIM3, DISABLE);
    COMP_Cmd(COMP_Selection_COMP2, DISABLE);
    cap_number = 0;
    while(1)
    {
    	scan_capture_handler();
    	if (cap_number == 359)
    	{
    		Delay_ms(30);
    		stop_capture();
    		start_send_result();
    	}
    }
#endif




    while(1)
    {
    	//start_send_result();
    	//uart_send_array(&line_buffer1, LINE_BUFFER_SIZE);
    	//Delay_ms(166);

    	set_adc_buf();
    	start_1deg_capture();
    	while (image_cap_status != CAPTURE_DONE){}
    	DWT_Delay(50);
    	//Delay_ms(1);
    	//capture_dma_stop();
    	//uart_send_array(&header[0], 3);
    	//uart_send_array(&line_buffer1[0], LINE_BUFFER_SIZE);
    	//tmp_range = centroid_finder(&line_buffer1[0]);
    	//Delay_ms(35);
    	//toggle_led();
    }
}
