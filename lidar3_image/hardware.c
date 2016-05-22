#include "hardware.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "adc_control.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_usart.h"

#define OPAMP_OFFET 2365

//#define OPAMP_OFFET 2350

void init_hardware(void)
{
	init_clock();
	DWT_Init();
	init_gpio();
	init_dac();
	init_uart();

    init_capture_gpio();
    adc_init();
    init_timer1();//sensor clk + adc capture


}

void init_clock(void)
{
	//switch to HSI
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
	while (RCC_GetSYSCLKSource() != 0x00) {}
	RCC_DeInit();
	//enable HSE
	RCC_HSEConfig(RCC_HSE_ON);
	RCC_WaitForHSEStartUp();
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) {}
	RCC_PLLConfig(RCC_PLLSource_PREDIV1,RCC_PLLMul_8); // PLL config 8*8=64
	RCC_PLLCmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
	//switch to HSE
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while (RCC_GetSYSCLKSource() != 0x08) {}
}

void init_gpio(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	//OUT PINS
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = (DATA_PIN|RES_PIN|SHT_PIN);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SENSOR_GPIO, &GPIO_InitStructure);

	GPIO_ResetBits(SENSOR_GPIO,(DATA_PIN|RES_PIN|SHT_PIN));

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LASER_PIN1|LASER_PIN2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LASER_GPIO, &GPIO_InitStructure);
}

void init_uart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = UART1_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

	GPIO_PinAFConfig(UART1_GPIO, UART1_TX_AFIO, GPIO_AF_7);
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 256000;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

inline void start_exposure(void)
{
	set_laser_on();
	asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");
	SENSOR_GPIO->BSRR = (RES_PIN|SHT_PIN);//set
	asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");
	asm("nop");asm("nop");asm("nop");asm("nop");
	SENSOR_GPIO->BRR = (RES_PIN);//reset

}

inline void stop_exposure(void)
{
	SENSOR_GPIO->BRR = SHT_PIN;
	//set_laser_off();
}

void do_capture(void)
{
	start_exposure();
	DWT_Delay(250);
	//stop_exposure();
	create_start_pulse();
}

void set_laser_on(void)
{
	GPIO_SetBits(LASER_GPIO,LASER_PIN1|LASER_PIN2);
}

void set_laser_off(void)
{
	GPIO_ResetBits(LASER_GPIO,LASER_PIN1|LASER_PIN2);
}

void set_led_on(void)
{
	GPIO_SetBits(LED_GPIO,LED_PIN);
}


void set_led_off(void)
{
	GPIO_ResetBits(LED_GPIO,LED_PIN);
}

void toggle_led(void)
{
	LED_GPIO->ODR^= LED_PIN;
}


void init_dac(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;

	DAC_DeInit();

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin = DAC1_PIN;
	GPIO_Init(DAC1_GPIO, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	DAC_StructInit(&DAC_InitStructure);
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	DAC_Cmd(DAC_Channel_1, ENABLE);
	//DAC_SetChannel1Data(DAC_Align_12b_R, 2360);
	DAC_SetChannel1Data(DAC_Align_12b_R, OPAMP_OFFET);
}

void Delay_ms(uint32_t ms)
{
	volatile uint32_t nCount;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);
	nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
	for (; nCount!=0; nCount--);
}


void DWT_Init(void)
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

uint32_t DWT_Get(void)
{
  return DWT->CYCCNT;
}

__inline uint8_t DWT_Compare(int32_t tp)
{
  return (((int32_t)DWT_Get() - tp) < 0);
}

void DWT_Delay(uint32_t us) // microseconds
{
  int32_t tp = DWT_Get() + us * (SystemCoreClock/1000000);
  while (DWT_Compare(tp));
}
