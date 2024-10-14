#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cstdio>
#include <cmath>

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

// Randomly initialized calibration matrix (A) and offset vector (b)
float A[3][3] =
    {{0.914256, -0.017041, -0.006617},
     {-0.017041, 0.913044, -0.008485},
     {-0.006617, -0.008485, 0.985565}};
// {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

float b[3] = {0.355573, 0.375152, 0.279745}; // Randomly initialized

class QMC5883L
{
public:
    QMC5883L(i2c_inst_t *i2c, uint8_t address) : i2c_(i2c), address_(address) {}

    void init()
    {
        // Initialize I2C communication
        i2c_init(i2c_, 400000);              // 400kHz I2C speed
        gpio_set_function(4, GPIO_FUNC_I2C); // SDA
        gpio_set_function(5, GPIO_FUNC_I2C); // SCL
        gpio_pull_up(4);
        gpio_pull_up(5);

        // Initialize QMC5883L
        uint8_t mode_config[2] = {REG_MODE, 0x01}; // Continuous mode, ODR = 200Hz, FS = 8G, OSR = 512
        int result = i2c_write_blocking(i2c_, address_, mode_config, 2, false);
        if (result < 0)
        {
            printf("Failed to initialize QMC5883L.\n");
        }
    }

    int16_t getX()
    {
        return readAxis(REG_X_LSB, REG_X_MSB);
    }

    int16_t getY()
    {
        return readAxis(REG_Y_LSB, REG_Y_MSB);
    }

    int16_t getZ()
    {
        return readAxis(REG_Z_LSB, REG_Z_MSB);
    }

private:
    i2c_inst_t *i2c_;
    uint8_t address_;

    int16_t readAxis(uint8_t lsb_reg, uint8_t msb_reg)
    {
        uint8_t buf[2];
        uint8_t reg = lsb_reg;

        // Set register address to read
        int result = i2c_write_blocking(i2c_, address_, &reg, 1, true);
        if (result < 0)
        {
            printf("Failed to set register address.\n");
            return 0; // Return 0 in case of error
        }

        // Read two bytes of data
        result = i2c_read_blocking(i2c_, address_, buf, 2, false);
        if (result < 0)
        {
            printf("Failed to read data.\n");
            return 0; // Return 0 in case of error
        }

        // Combine bytes into 16-bit value
        // printf("Nice_Data");
        return (int16_t)((buf[1] << 8) | buf[0]);
    }
};

// Calibration function using A * (x - b)
void calibrate(float x_mag, float y_mag, float z_mag, float &x_fin, float &y_fin, float &z_fin)
{
    // printf("Being called calibrate");
    float x_temp = x_mag - b[0]; // x - b_x
    float y_temp = y_mag - b[1]; // y - b_y
    float z_temp = z_mag - b[2]; // z - b_z

    // Perform matrix multiplication A * (x - b)
    x_fin = A[0][0] * x_temp + A[0][1] * y_temp + A[0][2] * z_temp;
    y_fin = A[1][0] * x_temp + A[1][1] * y_temp + A[1][2] * z_temp;
    z_fin = A[2][0] * x_temp + A[2][1] * y_temp + A[2][2] * z_temp;

    y_fin *= -1;
    z_fin *= -1;
}
/*
int main()
{
    stdio_init_all();

    QMC5883L compass(i2c0, QMC5883L_ADDR);
    compass.init();

    while (true)
    {
        int16_t raw_x = compass.getX();
        int16_t raw_y = compass.getY();
        int16_t raw_z = compass.getZ();

        // Convert raw data to float (assuming you need to convert based on your sensor's resolution)
        float x_mag = (float)raw_x / 12000;
        float y_mag = (float)raw_y / 12000;
        float z_mag = (float)raw_z / 12000;

        float x_calibrated, y_calibrated, z_calibrated;
        calibrate(x_mag, y_mag, z_mag, x_calibrated, y_calibrated, z_calibrated);

        // Print calibrated data
        printf("Bx: %.2f, By: %.2f, Bz: %.2f\n", x_calibrated, y_calibrated, z_calibrated);
        float norm = x_calibrated * x_calibrated + y_calibrated * y_calibrated + z_calibrated * z_calibrated;
        printf("%f", pow(norm, 0.5));
    }
}
// #include "pico/stdlib.h"
// #include "hardware/i2c.h"
// #include <cstdio>

// // QMC5883L I2C Address
// #define QMC5883L_ADDR 0x0D

// // Register Addresses
// #define REG_X_LSB 0x00
// #define REG_X_MSB 0x01
// #define REG_Y_LSB 0x02
// #define REG_Y_MSB 0x03
// #define REG_Z_LSB 0x04
// #define REG_Z_MSB 0x05
// #define REG_MODE 0x09

// class QMC5883L
// {
// public:
//     QMC5883L(i2c_inst_t *i2c, uint8_t address) : i2c_(i2c), address_(address) {}

//     void init()
//     {
//         // Initialize I2C communication
//         i2c_init(i2c_, 400000);              // 400kHz I2C speed
//         gpio_set_function(4, GPIO_FUNC_I2C); // SDA
//         gpio_set_function(5, GPIO_FUNC_I2C); // SCL
//         gpio_pull_up(4);
//         gpio_pull_up(5);

//         // Initialize QMC5883L
//         uint8_t mode_config[2] = {REG_MODE, 0x01}; // Continuous mode, ODR = 200Hz, FS = 8G, OSR = 512
//         int result = i2c_write_blocking(i2c_, address_, mode_config, 2, false);
//         if (result < 0)
//         {
//             printf("Failed to initialize QMC5883L.\n");
//         }
//     }

//     uint8_t readAxisLSB(uint8_t lsb_reg)
//     {
//         uint8_t lsb;
//         int result = i2c_write_blocking(i2c_, address_, &lsb_reg, 1, true);
//         if (result < 0)
//         {
//             printf("Failed to set LSB register address.\n");
//             return 0;
//         }
//         result = i2c_read_blocking(i2c_, address_, &lsb, 1, false);
//         if (result < 0)
//         {
//             printf("Failed to read LSB data.\n");
//             return 0;
//         }
//         return lsb;
//     }

//     uint8_t readAxisMSB(uint8_t msb_reg)
//     {
//         uint8_t msb;
//         int result = i2c_write_blocking(i2c_, address_, &msb_reg, 1, true);
//         if (result < 0)
//         {
//             printf("Failed to set MSB register address.\n");
//             return 0;
//         }
//         result = i2c_read_blocking(i2c_, address_, &msb, 1, false);
//         if (result < 0)
//         {
//             printf("Failed to read MSB data.\n");
//             return 0;
//         }
//         return msb;
//     }

//     int16_t getCombinedAxis(uint8_t lsb_reg, uint8_t msb_reg)
//     {
//         uint8_t lsb = readAxisLSB(lsb_reg);
//         uint8_t msb = readAxisMSB(msb_reg);
//         return (int16_t)((msb << 8) | lsb);
//     }

// private:
//     i2c_inst_t *i2c_;
//     uint8_t address_;
// };

// int main()
// {
//     stdio_init_all();

//     QMC5883L compass(i2c0, QMC5883L_ADDR); // Change i2c0 to i2c1 if you use i2c1
//     compass.init();

//     while (true)
//     {
//         // Read raw LSB and MSB for X, Y, Z axes
//         uint8_t x_lsb = compass.readAxisLSB(REG_X_LSB);
//         uint8_t x_msb = compass.readAxisMSB(REG_X_MSB);
//         uint8_t y_lsb = compass.readAxisLSB(REG_Y_LSB);
//         uint8_t y_msb = compass.readAxisMSB(REG_Y_MSB);
//         uint8_t z_lsb = compass.readAxisLSB(REG_Z_LSB);
//         uint8_t z_msb = compass.readAxisMSB(REG_Z_MSB);

//         // Combine LSB and MSB into 16-bit values for each axis
//         int16_t raw_x = (int16_t)((x_msb << 8) | x_lsb);
//         int16_t raw_y = (int16_t)((y_msb << 8) | y_lsb);
//         int16_t raw_z = (int16_t)((z_msb << 8) | z_lsb);

//         // Print raw values
//         printf("Raw X: LSB = %d, MSB = %d, Combined = %d\n", x_lsb, x_msb, raw_x);
//         printf("Raw Y: LSB = %d, MSB = %d, Combined = %d\n", y_lsb, y_msb, raw_y);
//         printf("Raw Z: LSB = %d, MSB = %d, Combined = %d\n", z_lsb, z_msb, raw_z);

//         sleep_ms(500); // Small delay for readability
//     }
// }
*/
