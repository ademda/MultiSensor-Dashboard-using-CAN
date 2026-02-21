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

void DHT11_Start(void);
uint8_t DHT11_Read(void);
#endif /* INC_DHT11_H_ */
