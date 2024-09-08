#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

// QMC5883L I2C Address
#define QMC5883L_ADDR 0x0D

// Register Addresses
#define REG_X_LSB 0x00
#define REG_X_MSB 0x01
#define REG_Y_LSB 0x02
#define REG_Y_MSB 0x03
#define REG_Z_LSB 0x04
#define REG_Z_MSB 0x05
#define REG_MODE 0x09

// Calibration constants
const float b1 = 0.073946f;
const float b2 = 0.067766f;
const float b3 = -0.206394f;
const float x1 = 1.083645f;
const float x2 = 0.044877f;
const float x3 = -0.035526f;
const float y1 = 0.044877f;
const float y2 = 1.253382f;
const float y3 = 0.012988f;
const float z1 = -0.035526f;
const float z2 = 0.012988f;
const float z3 = 1.163102f;

class QMC5883L {
public:
    QMC5883L(i2c_inst_t *i2c, uint8_t address) : i2c_(i2c), address_(address) {}

    void init() {
        // Initialize I2C communication
        i2c_init(i2c_, 400000); // 400kHz I2C speed
        gpio_set_function(4, GPIO_FUNC_I2C); // SDA
        gpio_set_function(5, GPIO_FUNC_I2C); // SCL
        gpio_pull_up(4);
        gpio_pull_up(5);

        // Initialize QMC5883L
        uint8_t mode_config[2] = {REG_MODE, 0x01}; // Continuous mode, ODR = 200Hz, FS = 8G, OSR = 512
        int result = i2c_write_blocking(i2c_, address_, mode_config, 2, false);
        if (result < 0) {
            printf("Failed to initialize QMC5883L.\n");
        }
    }

    int16_t getX() {
        return readAxis(REG_X_LSB, REG_X_MSB);
    }

    int16_t getY() {
        return readAxis(REG_Y_LSB, REG_Y_MSB);
    }

    int16_t getZ() {
        return readAxis(REG_Z_LSB, REG_Z_MSB);
    }

private:
    i2c_inst_t *i2c_;
    uint8_t address_;

    int16_t readAxis(uint8_t lsb_reg, uint8_t msb_reg) {
        uint8_t buf[2];
        uint8_t reg = lsb_reg;

        // Set register address to read
        int result = i2c_write_blocking(i2c_, address_, &reg, 1, true);
        if (result < 0) {
            printf("Failed to set register address.\n");
            return 0; // Return 0 in case of error
        }

        // Read two bytes of data
        result = i2c_read_blocking(i2c_, address_, buf, 2, false);
        if (result < 0) {
            printf("Failed to read data.\n");
            return 0; // Return 0 in case of error
        }

        // Combine bytes into 16-bit value
        return (int16_t)((buf[1] << 8) | buf[0]);
    }
};

// Calibration function
void calibrate(float x_mag, float y_mag, float z_mag, float &x_fin, float &y_fin, float &z_fin) {
    x_fin = (x_mag - b1) * x1 + (y_mag - b2) * x2 + (z_mag - b3) * x3;
    y_fin = (x_mag - b1) * y1 + (y_mag - b2) * y2 + (z_mag - b3) * y3;
    z_fin = (x_mag - b1) * z1 + (y_mag - b2) * z2 + (z_mag - b3) * z3;
}
int main() {
    stdio_init_all();

    QMC5883L compass(i2c0, QMC5883L_ADDR);
    compass.init();
    

    while (true) {
        int16_t raw_x = compass.getX();
        int16_t raw_y = compass.getY();
        int16_t raw_z = compass.getZ();

        // Convert raw data to float (assuming you need to convert based on your sensor's resolution)
        float x_mag = (float)raw_x;
        float y_mag = (float)raw_y;
        float z_mag = (float)raw_z;

        float x_calibrated, y_calibrated, z_calibrated;
        calibrate(x_mag/4096.0, y_mag/4096.0, z_mag/4096.0, x_calibrated, y_calibrated, z_calibrated);

        // Print calibrated data
        printf("Calibrated X: %.2f, Y: %.2f, Z: %.2f\n", x_calibrated, y_calibrated, z_calibrated);


    }
}
