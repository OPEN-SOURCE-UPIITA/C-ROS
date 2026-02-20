
#ifndef __RGB_SPI_H
#define __RGB_SPI_H

#include <stdio.h>
#include <stdint.h>
#include "packet.h"

#define Pixel_S1_NUM 2		//灯珠 RGB数量

#define CODE0 0xC0 // 0码, 发送的时间 1100 0000  根据不同的SCK适当调整
#define CODE1 0xF8 // 1码, 发送的时间 1111 1100

void WS2812b_Configuration(void);

void rgb_SendArray(void);

void set_id_rgb_color(struct Pixel* rgb);
void set_rgb_color(int num ,struct Pixel* rgb);

#endif //__RGB_SPI_H
