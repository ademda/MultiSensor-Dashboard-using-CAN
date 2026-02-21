/*
 * dht11.c
 *
 *  Created on: Feb 18, 2026
 *      Author: dalya
 */

#include "main.h"
#include "dht11.h"

dht11_t dht11;

void DHT11_Start(void)
{
	Set_Pin_Output (DHT11_GPIO_Port, DHT11_Pin);
	HAL_GPIO_WritePin (DHT11_GPIO_Port, DHT11_Pin, 0);
	delay_us (18000);
	Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);
}

uint8_t Check_Response(void)
{
	uint8_t Response = 0;
	delay_us (40);
	if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))
	{
		delay_us (80);
		if ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));

	return Response;
}

uint8_t DHT11_Read(void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));
		delay_us (40);
		if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))
		{
			i&= ~(1<<(7-j));
		}
		else i|= (1<<(7-j));
		while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));
	}
	return i;
}
