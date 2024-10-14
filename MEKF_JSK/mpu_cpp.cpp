#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "matrix.h"
// MPU6050 I2C Address
constexpr uint8_t MPU6050_ADDR = 0x68;

// MPU6050 Registers
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t GYRO_XOUT_H = 0x43;
constexpr uint8_t TEMP_OUT_H = 0x41;

// Sensitivity factors
constexpr float ACCEL_SENSITIVITY = 16384.0f;
constexpr float GYRO_SENSITIVITY_250 = 131.0f;

class MPU6050
{
public:
    MPU6050()
    {
        i2c_init(i2c0, 100000);
        gpio_set_function(4, GPIO_FUNC_I2C); // SDA
        gpio_set_function(5, GPIO_FUNC_I2C); // SCL
        gpio_pull_up(4);
        gpio_pull_up(5);

        mpu6050_init();
    }

    void read_data(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz, float &temp)
    {
        uint8_t buf[14];

        // Read accelerometer, gyroscope, and temperature data
        i2c_write_blocking(i2c0, MPU6050_ADDR, &ACCEL_XOUT_H, 1, true);
        i2c_read_blocking(i2c0, MPU6050_ADDR, buf, 14, false);

        ax = (buf[0] << 8) | buf[1];
        ay = (buf[2] << 8) | buf[3];
        az = (buf[4] << 8) | buf[5];
        gx = (buf[8] << 8) | buf[9];
        gy = (buf[10] << 8) | buf[11];
        gz = (buf[12] << 8) | buf[13];

        // Temperature in degrees Celsius
        int16_t raw_temp = (buf[6] << 8) | buf[7];
        temp = (raw_temp / 340.0f) + 35.0f;
    }

private:
    void mpu6050_init()
    {
        uint8_t data = 0x00;
        // Wake up the MPU6050
        i2c_write_blocking(i2c0, MPU6050_ADDR, &PWR_MGMT_1, 1, false);
        i2c_write_blocking(i2c0, MPU6050_ADDR, &data, 1, false);
    }
};