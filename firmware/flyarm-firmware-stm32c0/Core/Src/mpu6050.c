#include "mpu6050.h"
#include <stdint.h>
#include <math.h>

typedef struct{
    I2C_HandleTypeDef *i2c;
    uint8_t addr;
    int16_t gyroBias[3];
} mpu6050Handler_t;

static mpu6050Handler_t mpu6050Handler;

void mpu6050_writeReg(uint8_t reg, uint8_t data){
    if(mpu6050Handler.i2c == NULL) return;

    HAL_I2C_Mem_Write(mpu6050Handler.i2c, mpu6050Handler.addr, reg, 1, &data, 1, 100);
}

uint8_t mpu6050_readReg(uint8_t reg){
    if(mpu6050Handler.i2c == NULL) return 0;

    uint8_t data;
    HAL_I2C_Mem_Read(mpu6050Handler.i2c, mpu6050Handler.addr, reg, 1, &data, 1, 100);
    return data;
}

uint8_t mpu6050_whoami(void){
    return mpu6050_readReg(WHOAMI_REG);
}

void mpu6050_init(I2C_HandleTypeDef *i2cHandler, uint8_t chipAddr){
    mpu6050Handler.i2c = i2cHandler;
    mpu6050Handler.addr = chipAddr;
    for(uint8_t i=0; i<3; i++)
        mpu6050Handler.gyroBias[i] = 0;

    uint8_t temp;
    // Reset
    temp = 1 << 7;
    mpu6050_writeReg(PWR_MGMT_1_REG, temp);
    HAL_Delay(100);
    
    // Disable sleep, clock source to PLL from X axis gyro
    mpu6050_writeReg(PWR_MGMT_1_REG, 0x01);

    // DLPF to 20Hz
    mpu6050_writeReg(CONFIG_REG, 0x04);

    // Gyro full range 
    mpu6050_writeReg(GYRO_CONFIG_REG, 0x18);

    // Accel full range 
    mpu6050_writeReg(ACCEL_CONFIG_REG, 0x18);
}

void mpu6050_calibrateGyro(uint16_t samples){
    int32_t gyroBiasAccum[] = {0,0,0};
    for(uint16_t i=0; i<samples; i++){
        int16_t gyroRaw[3];
        int16_t accelRaw[3];
        mpu6050_readRaw(gyroRaw, accelRaw);
        
        gyroBiasAccum[0] += gyroRaw[0];
        gyroBiasAccum[1] += gyroRaw[1];
        gyroBiasAccum[2] += gyroRaw[2];

        HAL_Delay(2);
    }
    mpu6050Handler.gyroBias[0] = gyroBiasAccum[0] / samples;
    mpu6050Handler.gyroBias[1] = gyroBiasAccum[1] / samples;
    mpu6050Handler.gyroBias[2] = gyroBiasAccum[2] / samples;
}

void mpu6050_readRaw(int16_t *gyro, int16_t *accel){
    if(mpu6050Handler.i2c == NULL) return;
    
    uint8_t buffer[14];
    HAL_I2C_Mem_Read(mpu6050Handler.i2c, mpu6050Handler.addr, 0x3B, 1, buffer, 14, 100);
    
    accel[0] = (int16_t)(buffer[0]<<8 | buffer[1]);
    accel[1] = (int16_t)(buffer[2]<<8 | buffer[3]);
    accel[2] = (int16_t)(buffer[4]<<8 | buffer[5]);
    
    gyro[0] = (int16_t)(buffer[8]<<8 | buffer[9]);
    gyro[1] = (int16_t)(buffer[10]<<8 | buffer[11]);
    gyro[2] = (int16_t)(buffer[12]<<8 | buffer[13]);
}

void mpu6050_readScaled(float *gyroScaled, float *accelScaled){
    int16_t gyroRaw[3];
    int16_t accelRaw[3];
    mpu6050_readRaw(gyroRaw, accelRaw);

    for(uint8_t i=0; i<3; i++){
        gyroScaled[i] = (gyroRaw[i] - mpu6050Handler.gyroBias[i])/16.4f;
        accelScaled[i] = accelRaw[i]/2048.0f;
    }
}

void mpu6050_readGyro(int16_t *gyroRaw, int16_t *gyroBiased, float *gyroScaled, float *rpy, float dt){
    static float gyroAngleAccumX = 0.0f;
    static float gyroAngleAccumY = 0.0f;
    static float gyroAngleAccumZ = 0.0f;
    int16_t gyro[3];
    int16_t accel[3];
    mpu6050_readRaw(gyro, accel);

    for(uint8_t i=0; i<3; i++){
        gyroRaw[i] = gyro[i];
        gyroBiased[i] = gyro[i] - mpu6050Handler.gyroBias[i];
        gyroScaled[i] = gyroBiased[i]/16.4f;
    }

    gyroAngleAccumX += gyroScaled[0]*dt;
    gyroAngleAccumY += gyroScaled[1]*dt;
    gyroAngleAccumZ += gyroScaled[2]*dt;

    rpy[0] = gyroAngleAccumX;
    rpy[1] = gyroAngleAccumY;
    rpy[2] = gyroAngleAccumZ;
}

void mpu6050_readRoll(float *roll, float *dGyro, float alpha, float alphaGyro, float dt){
    static float rollAccum = 0.0f;
    static float dGyroAccum = 0.0f;

    float gyroScaled[3];
    float accelScaled[3];
    mpu6050_readScaled(gyroScaled, accelScaled);

    // From acc
    float rollAcc = atan2f(accelScaled[1], accelScaled[2]) * (180.0f / M_PI);

    // Fusion
    rollAccum = alpha * (rollAccum + gyroScaled[0]*dt) + (1.0f - alpha) * rollAcc;
    *roll = rollAccum;

    // Gyro
    dGyroAccum = alphaGyro * dGyroAccum + (1.0f - alphaGyro) * gyroScaled[0];
    *dGyro = dGyroAccum;
}