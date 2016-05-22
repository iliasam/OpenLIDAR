#ifndef _SPEED_FINDER_H
#define _SPEED_FINDER_H
#include "stdint.h"



#define ENC_NUM 15     //число делений датчика скорости
#define ROT_TIME 14500 //максимальное время 1 поворота(для предделителя 900)//80khz*14500 - 1/5.5 sec

#define ENC_ROT_TIME  (ROT_TIME/ENC_NUM)  //максимальное время 1 поворота датчика скорости
#define ENC_DEG       (uint16_t)(360/ENC_NUM) //число градусов в 1 делении датчика скорсти

void init_timer3(void);
void timer16_init(void);
uint8_t check_zero_point(uint16_t time);
void refresh_sync_timer(uint16_t time);

#endif
