#ifndef __AHOY_CRC_H__
#define __AHOY_CRC_H__

#include <cstdint>

#define CRC8_INIT               0x00
#define CRC8_POLY               0x01

#define CRC16_MODBUS_POLYNOM    0xA001
#define CRC16_NRF24_POLYNOM     0x1021

uint8_t AHOY_crc8(uint8_t buf[], uint8_t len);
uint16_t AHOY_crc16(uint8_t buf[], uint8_t len);
uint16_t AHOY_crc16nrf24(uint8_t buf[], uint16_t lenBits, uint16_t startBit = 0, uint16_t crcIn = 0xffff);

#endif /*__AHOY_CRC_H__*/
