#include "speed_finder.h"
#include "hardware.h"
#include "capture_control.h"
#include "adc_control.h"
#include "uart_func.h"

#include "stm32f30x_tim.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"

//timers frequency - 80kHz
#define TIM3_PRESCALER (uint16_t)(((SystemCoreClock) / 80000) - 1)//APB1 = HCLK/2*2
#define TIM16_PRESCALER (uint16_t)(((SystemCoreClock) / 80000) - 1)//APB2 = HCLK

//для датчика скорости
volatile uint16_t capture_old = 0;
volatile uint16_t capture_now = 0;

volatile uint16_t zero_old = 0;
volatile uint16_t zero_now = 0;
volatile uint16_t rot_period  = 0;//период вращения диска

volatile uint16_t enc_period;//время между срабатыванем компаратора

volatile uint16_t time_tmp = 0;//время поворота на 1 градус
volatile uint8_t  rot_num = 0;//число срабатываний датика скорости

extern volatile uint16_t cap_number;//число захваченных точек

volatile uint8_t sync_failed_flag = 0;//1 - сбой синхронизации по скорости (число секторов неверное)

volatile uint8_t overspeed_flag = 0;

//таймер синхронизации, отмечает новый градус
//начинает захват
void TIM1_UP_TIM16_IRQHandler(void)
{
  TIM16->SR &= ~TIM_SR_CC1IF;//сбросить флаг прерывания
  if (cap_number < 359)
  {
    cap_number++;
    set_adc_buf();
    start_1deg_capture();//start capture image for this angle
  }
}


//срабатывает со сигналу с компаратора (по энкодеру)
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		capture_old = capture_now;
		capture_now = TIM_GetCapture1(TIM3);

		//учет переполнения
		if (capture_now >= capture_old) {enc_period = capture_now - capture_old;} else {enc_period = 0xFFFF - capture_old + capture_now;}
		if (enc_period < ENC_ROT_TIME) {set_led_on();enc_period = ENC_ROT_TIME;overspeed_flag = 1;} else {set_led_off();overspeed_flag = 0;}
		time_tmp = enc_period / ENC_DEG;//время поворота на 1 градус //для 15 меток ENC_DEG = 24

		rot_num++;

		cap_number = rot_num*ENC_DEG;

		if (check_zero_point(time_tmp) == 1)//пересечение 0
		{
			//здесь enc_period удвоенный из-за закрашенного сектора
			enc_period = enc_period/2;
			time_tmp = enc_period / ENC_DEG;

			zero_old = zero_now;
			zero_now = TIM_GetCapture1(TIM3);
			if (zero_now >= zero_old) {rot_period = (zero_now - zero_old)/80;} //timer3 - 80khz
			else {rot_period = (0xFFFF - zero_old + zero_now)/80;}


			//if (rot_num != (ENC_NUM-1)) {set_led_on();sync_failed_flag = 1;} else {set_led_off();}
			//send_test_data(cap_number);

			if (rot_num != (ENC_NUM-1)) {sync_failed_flag = 1;}//(ENC_NUM-1) так как один закрашен
			stop_capture();
			refresh_sync_timer(time_tmp);
			if (sync_failed_flag == 0) {start_send_result();}//если оборот прошел нормально
			sync_failed_flag = 0;
			rot_num = 0;
			//ожидание захвата первого градуса

			//toggle_led();
			//send_test_data(enc_period);
		}

	    TIM_SetAutoreload(TIM16,time_tmp-1);
	    TIM_SetCompare1(TIM16,(time_tmp/2)-1);
	}
}

//нужнет для измерения скорости по энкодеру
//срабатывает по переднему фронту компаратора
void init_timer3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef      NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = TIM3_PRESCALER;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// channel1 - input capture
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 12;//uart produce some noise
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_ARRPreloadConfig(TIM3,ENABLE);

	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//NVIC_EnableIRQ(TIM3_IRQn);
	TIM_Cmd(TIM3, ENABLE);
}


//отвечает за синхронизацию и управление захватом(1 срабатывание - 1 градус)
void timer16_init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

  TIM_DeInit(TIM16);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Prescaler = TIM16_PRESCALER;
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;//Timing!
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 100;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM16, &TIM_OCInitStructure);

  TIM_ARRPreloadConfig(TIM16, ENABLE);
  TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);

  TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);

  TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
  //NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  //будет запущен позже
}

//если время в 1.5 раза превышает среднее - возвращает 1
//среднний период вычисляется здесь же
uint8_t check_zero_point(uint16_t time)
{
  static uint16_t times[4];
  static uint8_t pos = 0;//pos = 0-3
  uint16_t avr_time = 0;//средний период
  uint16_t result;

  times[pos] = time;
  pos++;
  pos&=3;
  avr_time = (times[0] + times[1] + times[2] + times[3]) / 4;
  time=time*4;
  result = time/avr_time;
  if (result >= 5) {return 1;} else {return 0;}//6/4 = 1.5
}

//останавлиает таймер синхронизации, СБРАСЫВАЕТ и загружает в него новое время поворота на 1 градус
//вызывается из обработчика прерываний TIM3 (энкодер) при пересечении нуля??????
void refresh_sync_timer(uint16_t time)
{

  TIM16->CR1&= ~TIM_CR1_CEN;//disable timer
  TIM_SetCounter(TIM16,0);
  TIM_SetAutoreload(TIM16,time);
  TIM_SetCompare1(TIM16,(time/4));
  TIM16->EGR |= TIM_EGR_UG;//update all registers and prescaler
  TIM16->CR1 |= TIM_CR1_CEN;//enable timer
}


