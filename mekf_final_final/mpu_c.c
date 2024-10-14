#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// MPU6050 Registers
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define TEMP_OUT_H 0x41

// Sensitivity factors
#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY_250 131.0f

// Function to initialize MPU6050
void mpu6050_init() {
    uint8_t data[2];
    data[0] = PWR_MGMT_1; // Register address
    data[1] = 0x00;      // Data to write

    // Wake up the MPU6050
    i2c_write_blocking(i2c0, MPU6050_ADDR, data, sizeof(data), false);
}

// Function to read and convert raw data from MPU6050
void read_mpu6050_data(
    float *ax_g, float *ay_g, float *az_g,
    float *gx_dps, float *gy_dps, float *gz_dps,
    float *temp
) {
    uint8_t buf[14];
    uint8_t reg = ACCEL_XOUT_H;

    // Read accelerometer, gyroscope, and temperature data
    i2c_write_blocking(i2c0, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, MPU6050_ADDR, buf, sizeof(buf), false);

    // Convert accelerometer data
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    *ax_g = ax / ACCEL_SENSITIVITY;
    *ay_g = ay / ACCEL_SENSITIVITY;
    *az_g = az / ACCEL_SENSITIVITY;

    // Convert gyroscope data
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
    *gx_dps = gx / GYRO_SENSITIVITY_250;
    *gy_dps = gy / GYRO_SENSITIVITY_250;
    *gz_dps = gz / GYRO_SENSITIVITY_250;

    // Convert temperature data
    int16_t raw_temp = (int16_t)((buf[6] << 8) | buf[7]);
    *temp = (raw_temp / 340.0f) + 35.0f;
}