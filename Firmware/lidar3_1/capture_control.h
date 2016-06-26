#ifndef __CAPTURE_CONTROL_H
#define __CAPTURE_CONTROL_H
#include "stdint.h"

//360 data + 2header + 1 status flags + 1 speed
#define PACKET_OFFSET (uint16_t)(2+1+1)
#define PACKET_LENGTH (uint16_t)(360+PACKET_OFFSET)

typedef struct
{
	uint16_t centroid;
	uint8_t low_power;
}centroid_result_type;

void stop_capture(void);
void timer17_init(void);
void start_1deg_capture(void);
void scan_capture_handler(void);
centroid_result_type centroid_finder(uint16_t *line_image);

int16_t test_symmetry(uint16_t *line_image, uint16_t max_pos);
uint16_t find_min(uint16_t *line_image, uint16_t start, uint16_t stop);

#endif

