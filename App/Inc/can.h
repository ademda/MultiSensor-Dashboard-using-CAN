/*
 * can.h
 *
 *  Created on: Feb 19, 2026
 *      Author: dalya
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>
#include "mpu6050.h"
#include "adc_sensor.h"
#include "dht11.h"

#define ADC_SENSOR_ID 0x005
#define DHT11_ID	  0x004
#define IMU_ID	  0x003

uint64_t Encode_Imu(MPU6050_t * imu_arg);
uint64_t Encode_Dht11(dht11_t* dht11_arg);
void Can_Init(CAN_TxHeaderTypeDef *adc_can_arg, CAN_TxHeaderTypeDef *dht11_can_arg, CAN_TxHeaderTypeDef *imu_can_arg);
void Send_Lum_Can(CAN_TxHeaderTypeDef *adc_can_arg, uint64_t val, uint32_t *Mailbox);
void Send_Imu_Can(CAN_TxHeaderTypeDef *imu_can_arg, uint64_t val, uint32_t *Mailbox);
void Send_dht11_Can(CAN_TxHeaderTypeDef *dht11_can_arg, uint64_t val, uint32_t *Mailbox);


#endif /* INC_CAN_H_ */
