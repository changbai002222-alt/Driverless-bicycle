#ifndef __UPPER_H__
#define __UPPER_H__

#include "main.h"
extern uint8_t buf[1];
extern int16_t delta_x_buf;
void upper_send(int steer);
void upper_send_data(uint8_t *buf,int len);
#endif

