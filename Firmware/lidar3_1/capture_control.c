#include "capture_control.h"
#include "speed_finder.h"
#include "adc_control.h"
#include "hardware.h"
#include "uart_func.h"

#include "stm32f30x_tim.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"

//timer frequency - 1MHz
#define EXP_TIM_PRESCALER (uint16_t)((SystemCoreClock / 1000000) - 1)//APB2 = HCLK
#define EXPOSURE_TIME (uint16_t)(250) //in usec

uint16_t res_buf0[PACKET_LENGTH];//result buffer (with centroids)
uint16_t res_buf1[PACKET_LENGTH];//result buffer

volatile uint8_t  res_buf_num = 0;//номер буфера, который «јѕќЋЌя≈“—я в данный момент, другой в этот момент передаетс€ в UART
//эти буферы переключаютс€ только при пересечении нул€

volatile uint16_t cap_number = 370;//число захваченных точек (370 - чтобы захват не начиналс€ сразу)

extern volatile cap_status_type image_cap_status;
extern volatile uint16_t* data_adc_p;//указатель на данные с лазером
volatile uint8_t image_done_flag = 0;//после обработки изображени€ установливаетс€ в 1 (защита от повторной обработки)

volatile uint32_t intagrate_left = 0;
volatile uint32_t intagrate_right = 0;



//экспозици€ начинаетс€
void start_1deg_capture(void)
{
	image_cap_status = NO_CAPTURE;
	start_exposure();
    TIM17->CR1 |= TIM_CR1_CEN;// Enable the TIM17 Counter (single pulse mode)
}

//exposure timer
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  TIM17->SR &= ~TIM_SR_UIF;//сбросить флаг прерывани€
  //exposure time ended
  //stop_exposure();//short exposure
  create_start_pulse();
  capture_dma_start();
  start_clk_timer();//begin to generate sensor CLK pulses
  image_cap_status = CAPTURE_IN_PROCESS;
  //proc_done_flag = 0;
  //next is DMA interrupt (all pixels values captured by ADC)
}

//отвечат за обработку захваченных изображений
void scan_capture_handler(void)
{
	uint16_t *tmp_data_pointer = data_adc_p;//дл€ защиты от переключени€ в процессе
	uint16_t tmp_cap_number = cap_number;//дл€ защиты от переключени€ в процессе
	uint8_t  tmp_res_buf_num = res_buf_num;
	centroid_result_type centroid_s;
	uint16_t res_value = 0;

	if ((image_cap_status == CAPTURE_DONE) && (image_done_flag == 0))//изображение захвачено и оно еще не обрабатывалось
	{
		centroid_s = centroid_finder(tmp_data_pointer);

		//if (tmp_res_buf_num == 0) centroid = 2000;

		res_value = centroid_s.centroid;
		if (centroid_s.low_power != 0) res_value+= 16384;

		if (tmp_res_buf_num == 0) {res_buf0[tmp_cap_number+PACKET_OFFSET] = res_value;} else {res_buf1[tmp_cap_number+PACKET_OFFSET] = res_value;}

		image_done_flag = 1;//изображение обработано
	}
	else if (image_cap_status != CAPTURE_DONE) {image_done_flag = 0;}//нечего обрабатывать
}


//прерывает захват изображени€
//вызываетс€ только при пересечении нул€
void stop_capture(void)
{
	capture_dma_stop();
	//tx_status = TRANSFER;
	res_buf_num^= 1;//изменить буфер результата
	clear_buffer();//очиcтить “≈ ”ў»… буфер перед началом записи в него
	cap_number = 0;
	//bad_point_flag = 1;
	//передача данных начнентс€ дальше
}



//отвечает за синхронизацию и управление захватом(1 срабатывание - 1 градус)
void timer17_init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

  TIM_DeInit(TIM17);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Prescaler = EXP_TIM_PRESCALER;
  TIM_TimeBaseStructure.TIM_Period = EXPOSURE_TIME;//1mhz*300 = 300us exposure time
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM17, ENABLE);
  TIM_SelectOnePulseMode(TIM17, TIM_OPMode_Single);//Counter stops counting at the next update event

  TIM_ClearITPendingBit(TIM17, TIM_IT_Update);

  TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);
  //NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 2);

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM17_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
  //будет запущен позже
}


#define AN_WIDTH 20 //половина ширины пика
//#define AN_WIDTH_RIGHT 20 //половина ширины пика
#define AN_WIDTH_RIGHT 8 //половина ширины пика

//выполн€ет поиск максимума в изображении
centroid_result_type centroid_finder(uint16_t *line_image)
{
	uint16_t i;
	uint16_t max_val = 0;
	uint16_t max_pos = 0;
	uint16_t start_pos;
	uint16_t stop_pos;

	int16_t start_pos_s;//signed
	int16_t stop_pos_s;//signed

	uint32_t summ = 0;
	uint32_t summ2 = 0;
	uint16_t value_offset = 0;//signal amplitude offset
	uint32_t tmp_centroid = 0;
	uint16_t val_threshold = 0;

	centroid_result_type result = {0,0};
	//используютс€ дл€ случа€ насыщени€
	uint16_t max_start_pos = 0;
	uint16_t max_stop_pos = 0;

	volatile int16_t sym_result = 0;

    for (i=10;i<LINE_BUFFER_SIZE;i++)
    {
      if (line_image[i] > max_val) {max_val = line_image[i]; max_pos = i;}//простой поиск максимума
      if (max_val > 1010) break;
    }

    start_pos_s = (int16_t)max_pos - (int16_t)AN_WIDTH;
    stop_pos_s =  (int16_t)max_pos + (int16_t)AN_WIDTH;


    if (max_val > 1010)//случай насыщени€ ADC
    {
    	i = LINE_BUFFER_SIZE-1;
    	while ((line_image[i] < 1010) && (i>10)) {i--;}//ищем конец максимума справа (переход от насыщени€ к нормальному уровню)

    	max_stop_pos = i;//первый максимум найден

    	while ((line_image[i] >= 1010) && (i>0)) {i--;}//ищем начало максимума справа
    	max_start_pos = i;//начало максимума найдено

    	start_pos_s = (int16_t)max_start_pos - (int16_t)AN_WIDTH;
    	stop_pos_s = (int16_t)max_stop_pos + (int16_t)AN_WIDTH;

    	max_pos = (max_start_pos  + max_stop_pos)/2;

    	if ((max_start_pos > 800) && (max_stop_pos > 900))
    	{
    		result.low_power = 1;
    	}
    }



    if (start_pos_s < 0) start_pos_s=0;
    if (stop_pos_s > (LINE_BUFFER_SIZE-1)) stop_pos_s = (LINE_BUFFER_SIZE-1);
    start_pos = (uint16_t)start_pos_s;
    stop_pos = (uint16_t)stop_pos_s;

    value_offset = line_image[start_pos];
    value_offset = find_min(line_image, start_pos, stop_pos);
    if (value_offset > 100) value_offset = 100;
    //val_threshold = (max_val - value_offset)/3 + value_offset;
    //val_threshold = max_val/3;
    //субпиксельный поиск максимума
    for (i = start_pos;i<stop_pos;i++)
    {
    	//if (line_image[i] > val_threshold)
    	if (1)
    	{
    	      summ = summ + (uint32_t)i * (uint32_t)(line_image[i] - value_offset);
    	      summ2 = summ2 +   (uint32_t)(line_image[i] - value_offset);
    	}
    }

    if (summ2 > 0)
    {
    	summ = summ*10;
    	tmp_centroid = (summ / summ2);
    }



    //sym_result = test_symmetry(line_image, max_pos);
    //if (sym_result>1) result.low_power = 1;

    if ((max_val - value_offset) < 30) result.low_power = 1;
    //if ((max_val) < 30) result.low_power = 1;


    if (tmp_centroid > 10240) {tmp_centroid = 10240;}

    result.centroid = (uint16_t)tmp_centroid;
    //result.centroid = (uint16_t)max_val;

    return result;
}

uint16_t find_min(uint16_t *line_image, uint16_t start, uint16_t stop)
{
	uint16_t i;
	uint16_t min_val = line_image[start];

    for (i = start;i<stop;i++)
    {
    	if (line_image[i] < min_val) min_val = line_image[i];
    }
	return min_val;
}

#define SYM_WIDTH 30 //половина ширины пика
int16_t test_symmetry(uint16_t *line_image, uint16_t max_pos)
{
	uint16_t start_pos;
	uint16_t stop_pos;
	uint16_t i;

	intagrate_left = 0;
	intagrate_right = 0;


    start_pos = max_pos - SYM_WIDTH;
    stop_pos =  max_pos + SYM_WIDTH;

    int16_t result = 0;

	if (max_pos < SYM_WIDTH) {start_pos = 0;}
	if (max_pos > (LINE_BUFFER_SIZE-1-SYM_WIDTH)) {stop_pos = 1023;}

    for (i = start_pos;i<=max_pos;i++)
    {
    	intagrate_left+= line_image[i];
    }

    for (i = max_pos;i<=stop_pos;i++)
    {
    	intagrate_right+= line_image[i];
    }

    if (intagrate_right > intagrate_left)
    {
    	result = (int16_t)(intagrate_right / intagrate_left);
    }
    else
    {
    	//result = ((int16_t)(intagrate_left / intagrate_right)) * (-1);
    	result = ((int16_t)(intagrate_left / intagrate_right));
    }

    return result;
}
