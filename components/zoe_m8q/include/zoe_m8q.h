#pragma once

#include <stdint.h>

#define ZOE_M8Q_DEFAULT_I2C_ADDR 0x42


struct zoe_m8q {
    uint8_t addr;
};