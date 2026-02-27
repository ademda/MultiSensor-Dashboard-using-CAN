/*
 * dht11.h
 *
 *  Created on: Feb 18, 2026
 *      Author: dalya
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#define DHT11_DATA_LENGHT 8

typedef struct {
	float hum;
	float temp;
}dht11_t;

void Set_Pin_Output(GPIO_TypeDef * GPIO_PORT, uint16_t GPIO_PIN);
void Set_Pin_Input(GPIO_TypeDef * GPIO_PORT, uint16_t GPIO_PIN);
void DHT11_Start(void);
uint8_t DHT11_Read(void);
#endif /* INC_DHT11_H_ */
