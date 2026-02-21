/*
 * can.c
 *
 *  Created on: Feb 19, 2026
 *      Author: dalya
 */
#include "can.h"
#include "dht11.h"
#include "mpu6050.h"
#include "adc_sensor.h"

extern CAN_HandleTypeDef hcan1;
extern uint16_t lum_value ;
extern dht11_t dht11;

CAN_TxHeaderTypeDef adc_can;
CAN_TxHeaderTypeDef dht11_can;
CAN_TxHeaderTypeDef imu_can;

uint32_t adc_Mailbox;
uint32_t dht11_Mailbox;
uint32_t imu_Mailbox;

uint64_t Encode_Imu(MPU6050_t * imu_arg){
	uint64_t ret_val = 0;
	ret_val = ((uint64_t)(imu_arg->pitch) << 32) + ((uint32_t)(imu_arg->roll));
	return ret_val;
}

uint64_t Encode_Dht11(dht11_t* dht11_arg){
	uint64_t ret_val = 0;
	ret_val = ((uint64_t)(dht11_arg->temp) << 8) + (uint32_t)(dht11_arg->hum);
	return ret_val;
}

void Can_Init(CAN_TxHeaderTypeDef *adc_can_arg, CAN_TxHeaderTypeDef *dht11_can_arg, CAN_TxHeaderTypeDef *imu_can_arg){
	//adc can header init
	adc_can_arg->DLC = ADC_SENSOR_DATA_LENGTH;
	adc_can_arg->StdId = ADC_SENSOR_ID;
	adc_can_arg->IDE = CAN_ID_STD;
	adc_can_arg->RTR = CAN_RTR_DATA;

	//dht11 can header init
	dht11_can_arg->DLC = DHT11_DATA_LENGHT;
	dht11_can_arg->StdId = DHT11_ID;
	dht11_can_arg->IDE = CAN_ID_STD;
	dht11_can_arg->RTR = CAN_RTR_DATA;

	//imu can header init
	imu_can_arg->DLC = IMU_DATA_LENGHT;
	imu_can_arg->StdId = IMU_ID;
	imu_can_arg->IDE = CAN_ID_STD;
	imu_can_arg->RTR = CAN_RTR_DATA;

}

void Send_Lum_Can(CAN_TxHeaderTypeDef *adc_can_arg, uint64_t val, uint32_t *Mailbox){
	if (HAL_CAN_AddTxMessage(&hcan1, adc_can_arg, val, Mailbox) != HAL_OK){
		Error_Handler();
	}
}
void Send_Imu_Can(CAN_TxHeaderTypeDef *imu_can_arg, uint64_t val, uint32_t *Mailbox){
	if (HAL_CAN_AddTxMessage(&hcan1, imu_can_arg, val, Mailbox) != HAL_OK){
		Error_Handler();
	}
}
void Send_dht11_Can(CAN_TxHeaderTypeDef *dht11_can_arg, uint64_t val, uint32_t *Mailbox){
	if (HAL_CAN_AddTxMessage(&hcan1, dht11_can_arg, val, Mailbox) != HAL_OK){
		Error_Handler();
	}
}
