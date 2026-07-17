#pragma once
#include <cstdint>

/**
 * @brief Tính CRC-16 (CCITT) theo lookup table
 * @param ptr Con trỏ dữ liệu (bao gồm cả 2 byte header 0x55 0xAA)
 * @param len Số byte cần tính (thường là 16 byte)
 */
uint16_t Get_CRC16(const uint8_t* ptr, uint16_t len);
