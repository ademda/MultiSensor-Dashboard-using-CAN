/*
 * sensors.h
 *
 *  Created on: Dec 28, 2025
 *      Author: dalya
 */

#ifndef INC_ADC_SENSOR_H_
#define INC_ADC_SENSOR_H_

#include <stdint.h>

#define ADC_SENSOR_DATA_LENGTH 2

void Get_Lum_Val_DMA();
void Get_Lum_Val_DMA_Complete();
#endif /* INC_ADC_SENSOR_H_ */
