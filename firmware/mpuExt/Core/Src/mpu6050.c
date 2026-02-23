/*
 * mpu6050.c
 *
 *  Created on: Dec 15, 2025
 *      Author: mateu
 */

#include "mpu6050.h"
#include "i2c.h"
#include <stdio.h>
#include <math.h>

#define CONFIG 0x1A

// Biasy (default ranges)
//>accel_x:-269
//>accel_y:304
//>accel_z:17909
//>gyro_x:-781
//>gyro_y:505
//>gyro_z:-36
//const int16_t accelConstBias[3] = {269, -304, 16384-17909};
//const int16_t gyroConstBias[3] = {781, -505, 36};

// Biasy (4g, 1000 dps)
// Accel: -50 15 8951
// Gyro: -195 130 -4
const int16_t accelConstBias[3] = {50, -15, 8192-8951};
const int16_t gyroConstBias[3] = {195, -130, 4};

const float a = 0.98f;

uint8_t mpu6050_ReadReg(uint8_t reg){
	uint8_t value = 0;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, &value, 1, 100);
	return value;
}

void mpu6050_WriteReg(uint8_t reg, uint8_t value){
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &value, 1, 100);
}

void mpu6050_ReadRawAccelGyro(int16_t *accel, int16_t *gyro){
	uint8_t buf[14];

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

	accel[0] = (int16_t)(buf[0] << 8 | buf[1]);
	accel[1] = (int16_t)(buf[2] << 8 | buf[3]);
	accel[2] = (int16_t)(buf[4] << 8 | buf[5]);

	gyro[0] = (int16_t)(buf[8] << 8 | buf[9]);
	gyro[1] = (int16_t)(buf[10] << 8 | buf[11]);
	gyro[2] = (int16_t)(buf[12] << 8 | buf[13]);
}

void mpu6050_ReadScaledAccelGyro(float *accelScaled, float *gyroScaled){
	int16_t accel[3], gyro[3];
	mpu6050_ReadRawAccelGyro(accel, gyro);

	// Scaling and biases
//	for(int i=0; i<3; i++){
//		accelScaled[i] = (accel[i] + accelConstBias[i]) / 16384.0f;
//		gyroScaled[i] = (gyro[i] + gyroConstBias[i]) / 131.0f;
//	}

	for(int i=0; i<3; i++){
			accelScaled[i] = (accel[i] + accelConstBias[i]) / 8192.0f;
			gyroScaled[i] = (gyro[i] + gyroConstBias[i]) / 32.8f;
		}
}

void mpu6050_ReadRawBias(int16_t *accelBias, int16_t *gyroBias){
	static int32_t accelAccum[3], gyroAccum[3];
	static int16_t counter=0;
	int16_t accel[3], gyro[3];

	if(counter < 1000){
		mpu6050_ReadRawAccelGyro(accel, gyro);
		for(int i=0; i<3; i++){
			accelAccum[i] += accel[i];
			gyroAccum[i] += gyro[i];
		}
		counter++;
		printf("Samples: %d\n", counter);
	}
	else{
		for(int i=0; i<3; i++){
			accelBias[i] = accelAccum[i]/1000;
			gyroBias[i] = gyroAccum[i]/1000;

			accelAccum[i]=0;
			gyroAccum[i]=0;

		}
		printf("Accel: %d %d %d\n", accelBias[0], accelBias[1], accelBias[2]);
		printf("Gyro: %d %d %d\n", gyroBias[0], gyroBias[1], gyroBias[2]);
		counter = 0;
	}
}

void mpu6050_ReadRoll(float *retRoll, float *retDGyro, float *accelScaled, float *gyroScaled){
	static float rollGyro = 0;
	static float roll = 0;
	static float DGyro = 0;
	float alpha = 0.3f;

	float accel[3], gyro[3];
	mpu6050_ReadScaledAccelGyro(accel, gyro);

	// Accelerometer
	float rollAcc = atan2f(accel[1], accel[2]) * (180.0f / M_PI);
	retRoll[0] = rollAcc;

	// Gyro
	rollGyro = rollGyro + gyro[0] * 0.01f;
	retRoll[1] = rollGyro;

	// Sensor fusion
	roll = a*(roll+(gyro[0]*0.01f)) + (1.0f-a)*rollAcc;
	retRoll[2] = roll;

	// Gyro for D
	DGyro = alpha * DGyro + (1.f-alpha) * gyro[0];
	*retDGyro = DGyro;

	// Scaled values for debug
	for(int i=0; i<3; i++){
		accelScaled[i] = accel[i];
		gyroScaled[i] = gyro[i];
	}
}

void mpu6050_Init(void){
	mpu6050_WriteReg(PWR_MGMT_1, 0x01);
	HAL_Delay(10);

	//Gyro +-1000 dps
	mpu6050_WriteReg(0x1B, 0x10);

	//Accel +- 4g
	mpu6050_WriteReg(0x1C, 0x08);

	HAL_Delay(100);
}
