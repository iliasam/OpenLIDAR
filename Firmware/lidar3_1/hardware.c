#include "hardware.h"
#include "speed_finder.h"
#include "adc_control.h"
#include "capture_control.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_comp.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_dbgmcu.h"

#define UART_SPEED 57600
//#define UART_SPEED 256000

//#define OPAMP_OFFET 2365
#define OPAMP_OFFET 1800

#define ENCODER_DAC 2000

void init_hardware(void)
{
	init_clock();
	DWT_Init();
	init_gpio();
	init_dac();
	init_uart();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	init_uart_DMA();
    init_capture_gpio();
    adc_init();
    init_timer1();//sensor clk + adc capture
    init_comparator();//used for encoder
    init_timer3();//used to measure speed
    timer16_init();//used to generate 1 deg pulses
    timer17_init();//uset to generate exposure pulse

    DBGMCU_APB1PeriphConfig(DBGMCU_TIM3_STOP,ENABLE);

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
	RCC_PCLK1Config(RCC_HCLK_Div2);//max 36MHz
	RCC_PCLK2Config(RCC_HCLK_Div1);//max 72MHz
	RCC_PLLConfig(RCC_PLLSource_PREDIV1,RCC_PLLMul_9); // PLL config 8*9=72
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
	USART_InitStructure.USART_BaudRate = UART_SPEED;
	USART_Init(USART1, &USART_InitStructure);

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	USART_Cmd(USART1, ENABLE);
}

void init_uart_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel4);//master
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->TDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;//not used now
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 360;//not used now
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	//прерывания не нужны
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
	SENSOR_GPIO->BRR = (RES_PIN);//reset bit

}

inline void stop_exposure(void)
{
	SENSOR_GPIO->BRR = SHT_PIN;
	//set_laser_off();
}

void do_capture(void)
{
	start_exposure();
	DWT_Delay(300);
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

	DAC_Cmd(DAC_Channel_1, ENABLE);//channel 1 used to set opamp offset
	DAC_SetChannel1Data(DAC_Align_12b_R, OPAMP_OFFET);

	DAC_Cmd(DAC_Channel_2, ENABLE);//channel 2 used as ref. voltage for encoder comparator
	DAC_SetChannel2Data(DAC_Align_12b_R, ENCODER_DAC);
}

void Delay_ms(uint32_t ms)
{
	volatile uint32_t nCount;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);
	nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
	for (; nCount!=0; nCount--);
}

void init_comparator(void)
{
	COMP_InitTypeDef COMP_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ENCODER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(ENCODER_GPIO, &GPIO_InitStructure);

	/* COMP Peripheral clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	COMP_StructInit(&COMP_InitStructure);

	COMP_InitStructure.COMP_NonInvertingInput = COMP_NonInvertingInput_IO1;//pa7 - comp2
	COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_DAC1;
	COMP_InitStructure.COMP_Output = COMP_Output_TIM3IC1;
	//COMP_InitStructure.COMP_Output =  COMP_Output_TIM2IC4;
	COMP_InitStructure.COMP_Mode = COMP_Mode_MediumSpeed;
	COMP_InitStructure.COMP_Hysteresis = COMP_Hysteresis_Medium;
	COMP_Init(COMP_Selection_COMP2, &COMP_InitStructure);

	COMP_Cmd(COMP_Selection_COMP2, ENABLE);
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
