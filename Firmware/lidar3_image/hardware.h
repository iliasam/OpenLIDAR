#ifndef _HARDWARE_H
#define _HARDWARE_H

#include <stdint.h>

#define LED_GPIO GPIOB
#define LED_PIN  GPIO_Pin_9

#define CLK_GPIO GPIOA
#define CLK_PIN  GPIO_Pin_8

#define ADC1_GPIO GPIOA
#define ADC2_GPIO GPIOB
#define ADC1_PIN  GPIO_Pin_2
#define ADC2_PIN  GPIO_Pin_2

#define DAC1_GPIO GPIOA
#define DAC1_PIN  GPIO_Pin_4

#define SENSOR_GPIO GPIOB
#define DATA_PIN  GPIO_Pin_12
#define RES_PIN   GPIO_Pin_13
#define SHT_PIN   GPIO_Pin_14

#define LASER_GPIO 	 GPIOB
#define LASER_PIN1   GPIO_Pin_3
#define LASER_PIN2   GPIO_Pin_4

#define UART1_GPIO 	 GPIOB
#define UART1_TX_PIN   GPIO_Pin_6
#define UART1_TX_AFIO  GPIO_PinSource6

void init_hardware(void);

void init_gpio(void);
void init_clock(void);
void init_dac(void);
void init_uart(void);

inline void start_exposure(void);
inline void stop_exposure(void);
void do_capture(void);

void set_laser_on(void);
void set_laser_off(void);


void set_led_on(void);
void set_led_off(void);
void toggle_led(void);

void Delay_ms(uint32_t ms);

void DWT_Init(void);
uint32_t DWT_Get(void);
void DWT_Delay(uint32_t us);


#endif
