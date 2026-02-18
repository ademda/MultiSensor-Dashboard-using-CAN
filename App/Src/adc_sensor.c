/*
 * sensors.c
 *
 *  Created on: Dec 28, 2025
 *      Author: dalya
 */

#include "adc_sensor.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

uint16_t lum_value = 0;
uint8_t lum_val_ready = 0;

void Get_Lum_Val_DMA(){
	HAL_ADC_Start_DMA(&hadc1, (void *)&lum_value, 2);
	lum_val_ready = 0;
}

void Get_Lum_Val_DMA_Complete(){
	lum_val_ready = 1;
}

