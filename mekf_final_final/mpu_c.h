#ifndef MPU_C_H
#define MPU_C_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// MPU6050 Registers
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// Sensitivity factors
#define ACCEL_SENSITIVITY 16384.0f

// Function to initialize MPU6050
void mpu6050_init();

// Function to read accelerometer data from MPU6050
void read_mpu6050_data(float &ax_g, float &ay_g, float &az_g);

#endif // MPU6050_H
