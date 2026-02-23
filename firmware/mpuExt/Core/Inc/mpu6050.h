/*
 * mpu6050.h
 *
 *  Created on: Dec 15, 2025
 *      Author: mateu
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>

// Rejestry
#define MPU6050_ADDR        0x68 << 1
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1          0x6B
#define ACCEL_XOUT_H        0x3B


// Reading data from reg
uint8_t mpu6050_ReadReg(uint8_t reg);

// Writing data to reg
void mpu6050_WriteReg(uint8_t reg, uint8_t value);

// Reading raw accelerometer and gyro data
void mpu6050_ReadRawAccelGyro(int16_t *accel, int16_t *gyro);

// Reading accelerometer and gyro data with adding bias and scaling
void mpu6050_ReadScaledAccelGyro(float *accel, float *gyro);

// Reading sensor bias
void mpu6050_ReadRawBias(int16_t *accelBias, int16_t *gyroBias);

// Reading pendulum roll angle - after sensor fusion - data for regulator, reading retDGyro for D part in PID controller
void mpu6050_ReadRoll(float *roll, float *retDGyro, float *accelScaled, float *gyroScaled);

// Sensor initialization
void mpu6050_Init(void);

#endif /* INC_MPU6050_H_ */
