#include "hardware.h"
#include "uart_func.h"
#include "adc_control.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_tim.h"

extern volatile uint16_t line_buffer[LINE_BUFFER_SIZE];
RCC_ClocksTypeDef RCC_Clocks;

uint16_t header[3] = {0xAABB,0xCCDD,0xEEFF};

int main(void)
{
	init_hardware();

    RCC_GetClocksFreq (&RCC_Clocks);

    start_clk_timer();
    //set_led_on();

    //SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

    Delay_ms(500);
    while(1)
    {
    	do_capture();
    	capture_dma_start();
    	start_clk_timer();
    	Delay_ms(1);
    	uart_send_array(&header[0], 3);
    	uart_send_array(&line_buffer, LINE_BUFFER_SIZE);
    	//Delay_ms(35);
    	//toggle_led();
    }
}
