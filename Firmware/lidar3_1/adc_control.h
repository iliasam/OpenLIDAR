#ifndef __ADC_CONTROL_H
#define __ADC_CONTROL_H

#define LINE_BUFFER_SIZE (uint16_t)1024

typedef enum
{
  NO_CAPTURE = 0,
  CAPTURE_IN_PROCESS,
  CAPTURE_DONE
}cap_status_type;//image capture status

void init_clk(void);
void adc_init(void);
void init_timer1(void);

void capture_dma_start(void);
void capture_dma_stop(void);

void start_clk_timer(void);
void init_capture_gpio(void);

void create_start_pulse(void);

void start_1deg_capture(void);
void set_adc_buf(void);



#endif 

