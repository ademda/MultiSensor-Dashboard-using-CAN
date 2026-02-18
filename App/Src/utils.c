/*
 * utils.c
 *
 *  Created on: Feb 18, 2026
 *      Author: dalya
 */
#include "main.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim1;


void delay_us(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}
