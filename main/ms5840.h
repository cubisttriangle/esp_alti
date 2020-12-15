#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MS5840_I2C_ADDR 0x76
#define MS5840_PROM_START_ADDR 0xA0
#define MS5840_CMD_D1_4096 0x48   // 4K pressure
#define MS5840_CMD_D2_4096 0x58   // 4k temperature

struct ms5840_data
{
    // List of calibration values stored in PROM.
    // 0: factory and product info
    // 1: pressure sensitivity | SENS_t1
    // 2: pressure offset | OFF_t1
    // 3: temperature coefficient of pressure sensitivity | TCS
    // 4: temperature coefficient of pressure offset | TCO
    // 5: reference temperature | T_ref
    // 6: temperature coefficient of temperature | TEMPSENS
    uint16_t c[7];

    // Digital pressure reading.
    uint32_t d1;

    // Digital temperature reading.
    uint32_t d2;
};

struct ms5840_results
{
    // First order calculations
    int32_t dt;   // Difference between actual and reference temperature
    int32_t temp; // Actual temperature (-40C...85C +/- 0.01C)
    int64_t off;  // Offset pressure at actual temperature
    int64_t sens; // Sensitivity at actual temperature
    int32_t p;    // Temperature compensated pressure (10mbar...1200mbar +/- 0.01mbar)

    // Second order (temperature-dependent) calculations
    int32_t ti;
    int64_t offi;
    int64_t sensi;

    // Final calculations
    int32_t temp2;
    int64_t off2;
    int64_t sens2;
};

void ms5840_calc_results(struct ms5840_data* dev, struct ms5840_results* results);
uint8_t ms5840_factory_bits(struct ms5840_data* dev);
uint8_t ms5840_product_type(struct ms5840_data* dev);
int8_t ms5840_crcs_match(struct ms5840_data* dev);
uint8_t ms5840_expected_crc(struct ms5840_data* dev);
uint8_t ms5840_calculated_crc(struct ms5840_data* dev);
const char* ms5840_product_name_from_product_type(uint8_t product_type);
void ms5840_load_example_data(struct ms5840_data* dev);

#ifdef __cplusplus
}
#endif