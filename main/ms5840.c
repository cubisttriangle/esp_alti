#include "ms5840.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void ms5840_calc_results(struct ms5840_data* dev, struct ms5840_results* results)
{
    // Max values for calculation results:
    // P_min = 10mbar, P_max = 2000mbar
    // T_min = -40C, T_max = 85C, T_ref = 20C

    // First order calculations
    results->dt = dev->d2 - dev->c[5] * pow(2, 8);
    results->temp = 2000 + results->dt * dev->c[6] / pow(2, 23);
    results->off = dev->c[2] * pow(2, 17) + results->dt * dev->c[4] / pow(2, 6);
    results->sens = dev->c[1] * pow(2, 16) + results->dt * dev->c[3] / pow(2, 7);
    results->p = (dev->d1 * results->sens / pow(2, 21) - results->off) / pow(2, 15);

    // Second order calculations
    if (results->temp > 2000)
    {
        results->ti = 0;
        results->offi = 0;
        results->sensi = 0;
    }
    else if (results->temp > 1000)
    {
        results->ti = 12 * pow(results->dt, 2) / pow(2, 35);
        results->offi = 30 * pow((results->temp - 2000), 2) / pow(2, 8);
        results->sensi = 0;
    }
    else
    {
        results->ti = 14 * pow(results->dt, 2) / pow(2, 35);
        results->offi = 35 * pow((results->temp - 2000), 2) / pow(2, 3);
        results->sensi = 63 * pow((results->temp - 2000), 2) / pow(2, 5);
    }

    // Final calculations
    results->temp2 = (results->temp - results->ti) / 100;
    results->off2 = results->off - results->offi;
    results->sens2 = results->sens - results->sensi;
}

int8_t ms5840_crcs_match(struct ms5840_data* dev)
{
    uint8_t expected = ms5840_expected_crc(dev);
    uint8_t calculated = ms5840_calculated_crc(dev);
    printf("CRC expected: %d, calculated: %d\n", (int)expected, (int)calculated);
    return expected == calculated;
}

uint8_t ms5840_expected_crc(struct ms5840_data* dev)
{
    return dev->c[0] >> 12;
}

uint8_t ms5840_calculated_crc(struct ms5840_data* dev)
{
    int cnt; // simple counter
    unsigned int rem = 0; // crc remainder
    unsigned char bit;
    // TODO: dont modify dev!
    dev->c[0]=((dev->c[0]) & 0x0FFF); // CRC byte is replaced by 0
    dev->c[7]=0; // Subsidiary value, set to 0
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    { // choose LSB or MSB
        if (cnt%2==1) rem ^= (unsigned short) ((dev->c[cnt>>1]) & 0x00FF);
        else rem ^= (unsigned short) (dev->c[cnt>>1]>>8);
        for (bit = 8; bit > 0; bit--)
        {
            if (rem & (0x8000)) rem = (rem << 1) ^ 0x3000;
            else rem = (rem << 1);
        }
    }
    rem = ((rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
    return (rem ^ 0x00);
}

uint8_t ms5840_factory_bits(struct ms5840_data* dev)
{
    return dev->c[0] & 0x1F;
}

uint8_t ms5840_product_type(struct ms5840_data* dev)
{
    return (dev->c[0] >> 5) & 0x7F;
}

const char* product_name_from_product_type(uint8_t product_type)
{
    switch (product_type)
    {
        case 0x00:
            return "MS5840-02BA21";
        case 0x24:
            return "MS5840-02BS36";
        default:
            return "UNKNOWN";
    }
}

void ms5840_load_example_data(struct ms5840_data* dev)
{
    dev->c[0] = 0; // TODO: make this some realistic value.
    dev->c[1] = 46372;
    dev->c[2] = 43981;
    dev->c[3] = 29059;
    dev->c[4] = 27842;
    dev->c[5] = 31553;
    dev->c[6] = 28165;
    dev->d1 = 6464444;
    dev->d2 = 8077636;
}
