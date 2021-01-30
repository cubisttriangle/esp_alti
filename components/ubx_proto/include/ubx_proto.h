#pragma once

#include <stdint.h>

// https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf

// Given the input buff, which consists of class id through payload, calc a and b checksum vals.
void ubx_checksum(uint8_t* buff, uint32_t buff_len, uint8_t* ck_a, uint8_t* ck_b);

// Char 1 and 2 of message preamble.
#define UBX_PREAMBLE_C1 0xB5
#define UBX_PREAMBLE_C2 0x62

#define UBX_CFG_CLASS 0x06
#define UBX_CFG_DAT 0x06
#define UBX_CFG_DAT_LEN 52

#define UBX_MON_CLASS 0x0A
#define UBX_MON_VER 0x04

#define UBX_SEC_CLASS 0x27
#define UBX_SEC_UNIQID 0x03
#define UBX_SEC_UNIQID_LEN 0x09
