#include "ubx_proto.h"

#include <stdio.h>

void ubx_checksum(uint8_t* buff, uint32_t buff_len, uint8_t* ck_a, uint8_t* ck_b)
{
    *ck_a = *ck_b = 0;

    for (int i = 0; i < buff_len; ++i)
    {
        *ck_a = *ck_a + buff[i];
        *ck_b = *ck_b + *ck_a;
    }

    printf("ck_a: %02x, ck_b: %02x\n", *ck_a, *ck_b);
}