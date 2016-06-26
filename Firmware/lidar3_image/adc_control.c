#include "stm32f30x.h"
#include "adc_control.h"
#include "hardware.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_misc.h"


volatile uint16_t line_buffer[LINE_BUFFER_SIZE];


//ADC1 & ADC2 for image capture
void adc_init(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;

  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
  // Configure ADC Channel 12 pin as analog input
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ADC1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(ADC1_GPIO, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ADC2_PIN;
  GPIO_Init(ADC2_GPIO, &GPIO_InitStructure);

  DMA_DeInit(DMA1_Channel1);//master
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1_2->CDR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&line_buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = LINE_BUFFER_SIZE/2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_ClearITPendingBit(DMA1_IT_TC1);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
  
  DMA_Cmd(DMA1_Channel1, ENABLE);

  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  ADC_VoltageRegulatorCmd(ADC2, ENABLE);
  DWT_Delay(20);
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1) != RESET);

  ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC2);
  while(ADC_GetCalibrationStatus(ADC2) != RESET);

  // ADC Common configuration *************************************************
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Interleave;
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv1;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 1;//+1
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);


  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;//ext trigger
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_1;      //ext1 - TIM1_CC2
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
  ADC_InitStructure.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  //ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;//test
  ADC_Init(ADC2, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_1Cycles5);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_1Cycles5);

  ADC_DMAConfig(ADC1, ADC_DMAMode_OneShot);
  //ADC_DMAConfig(ADC2, ADC_DMAMode_OneShot);
  ADC_DMACmd(ADC1, ENABLE);
  //ADC_DMACmd(ADC2, ENABLE);

  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);

  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));

  ADC_StartConversion(ADC2);
  ADC_StartConversion(ADC1);

  /*
  // Enable DMA request after last transfer (multi-ADC mode) ******************
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
*/
}


void DMA1_Channel1_IRQHandler(void)
{
  //TIM_ForcedOC4Config(TIM2, TIM_ForcedAction_Active);
  //GPIOD->ODR&= ~GPIO_Pin_15;
  TIM1->CR1 &= (uint16_t)~TIM_CR1_CEN;//stop timer
  TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
  if(DMA_GetITStatus(DMA1_IT_TC1))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);
    DMA_Cmd(DMA1_Channel1, DISABLE);

    //SENSOR_GPIO->BSRR = (RES_PIN);//set
    stop_exposure();
  }
}



//configure DMA
void capture_dma_start(void)
{
  DMA1_Channel1->CNDTR = LINE_BUFFER_SIZE/2;//two adc give one 32-bit "sample"
  DMA1_Channel1->CMAR = (uint32_t)&line_buffer;
  DMA_ClearITPendingBit(DMA1_IT_TC1);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);//по прерыванию останавливается таймер 1
  DMA_Cmd(DMA1_Channel1, ENABLE);
}


//запускает таймер 1 - он формирует clk для линейкии и управляет ADC
void start_clk_timer(void)
{
  TIM_SetCounter(TIM1, 0);
  TIM_SelectOCxM(TIM1,TIM_Channel_1,TIM_OCMode_PWM1);//для Forced
  TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Enable);//для Forced
  TIM_Cmd(TIM1,ENABLE);
}

//must be called before "start_clk_timer"
void create_start_pulse(void)
{
	SENSOR_GPIO->BSRR = DATA_PIN;//set
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_Active);//clk high
	asm("nop");asm("nop");asm("nop");asm("nop");
	SENSOR_GPIO->BRR = DATA_PIN;//reset
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//clk low
	asm("nop");asm("nop");asm("nop");asm("nop");
}


//формирует clk для линейки и управляет ADC
void init_timer1(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = 11;//72M/(7+1) = 8
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  //4MHZ - adc trigger
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 7;//PWM DUTY
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM1,&TIM_OCInitStructure);
  //TIM_SelectOutputTrigger2(TIM1, TIM_TRGOSource_OC2Ref);//for adc

  //8MHZ - ext clk generator
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 5;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM1,&TIM_OCInitStructure);
 
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Reset);//not adc
  TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Enable);
  TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Enable);
  TIM_CtrlPWMOutputs(TIM1 , ENABLE);

  TIM_ARRPreloadConfig(TIM1,ENABLE);
}


void init_capture_gpio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//clk pin
  
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin= CLK_PIN;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(CLK_GPIO,&GPIO_InitStructure);
  
  GPIO_PinAFConfig(CLK_GPIO,GPIO_PinSource8,GPIO_AF_6);
}
