// qmc.hpp
#ifndef QMC_H
#define QMC_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class QMC5883L
{
public:
    QMC5883L(i2c_inst_t *i2c, uint8_t address);
    void init();
    int16_t getX();
    int16_t getY();
    int16_t getZ();

private:
    i2c_inst_t *i2c_;
    uint8_t address_;
    int16_t readAxis(uint8_t lsb_reg, uint8_t msb_reg);
};

#endif // QMC_H
