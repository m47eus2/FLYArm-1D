#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "i2c.h"

#define WHOAMI_REG 0x75
#define PWR_MGMT_1_REG 0x6B 
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C

void mpu6050_writeReg(uint8_t reg, uint8_t data);
uint8_t mpu6050_readReg(uint8_t reg);

uint8_t mpu6050_whoami(void);
void mpu6050_init(I2C_HandleTypeDef *i2cHandler, uint8_t chipAddr);
void mpu6050_calibrateGyro(uint16_t samples);
void mpu6050_readRaw(int16_t *gyro, int16_t *accel);
void mpu6050_readScaled(float *gyroScaled, float *accelScaled);
void mpu6050_readGyro(int16_t *gyroRaw, int16_t *gyroBiased, float *gyroScaled, float *rpy, float dt);
void mpu6050_readRoll(float *roll, float *dGyro, float alpha, float alphaGyro, float dt);

#endif