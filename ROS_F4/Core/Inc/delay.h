#ifndef __DELAY_H
#define __DELAY_H

#include "stdint.h"
#include "stm32f407xx.h"

void delay_us(uint32_t nus);
void delay_ms(uint32_t nms);

#endif

