#include "crc.h"

uint8_t AHOY_crc8(uint8_t buf[], uint8_t len) {
    uint8_t crc = CRC8_INIT;
    for(uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for(uint8_t b = 0; b < 8; b ++) {
            crc = (crc << 1) ^ ((crc & 0x80) ? CRC8_POLY : 0x00);
        }
    }
    return crc;
}

uint16_t AHOY_crc16(uint8_t buf[], uint8_t len) {
    uint16_t crc = 0xffff;
    uint8_t shift = 0;

    for(uint8_t i = 0; i < len; i ++) {
        crc = crc ^ buf[i];
        for(uint8_t bit = 0; bit < 8; bit ++) {
            shift = (crc & 0x0001);
            crc = crc >> 1;
            if(shift != 0)
                crc = crc ^ 0xA001;
        }
    }
    return crc;
}

uint16_t AHOY_crc16nrf24(uint8_t buf[], uint16_t lenBits, uint16_t startBit, uint16_t crcIn) {
    uint16_t crc = crcIn;
    uint8_t idx, val = buf[(startBit >> 3)];

    for(uint16_t bit = startBit; bit < lenBits; bit ++) {
        idx = bit & 0x07;
        if(0 == idx)
            val = buf[(bit >> 3)];
        crc ^= 0x8000 & (val << (8 + idx));
        crc = (crc & 0x8000) ? ((crc << 1) ^ CRC16_NRF24_POLYNOM) : (crc << 1);
    }

    return crc;
}