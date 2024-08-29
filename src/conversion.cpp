/**
 * @file Conversion.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief 
 * @version 1.0
 * @date 2024-08-23
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#include "util/conversion.hpp"

void Conversion::uint16ToBigEndian(uint16_t value, uint8_t* buffer) {
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}

uint16_t Conversion::bigEndianToUint16(const uint8_t* buffer) {
    return (static_cast<uint16_t>(buffer[0]) << 8) |
           static_cast<uint16_t>(buffer[1]);
}

void Conversion::uint32ToBigEndian(uint32_t value, uint8_t* buffer) {
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;
}

uint32_t Conversion::bigEndianToUint32(const uint8_t* buffer) {
    return (static_cast<uint32_t>(buffer[0]) << 24) |
           (static_cast<uint32_t>(buffer[1]) << 16) |
           (static_cast<uint32_t>(buffer[2]) << 8)  |
           static_cast<uint32_t>(buffer[3]);
}

void Conversion::uint16ToLittleEndian(uint16_t value, uint8_t* buffer) {
    buffer[1] = (value >> 8) & 0xFF;
    buffer[0] = value & 0xFF;
}

void Conversion::uint32ToLittleEndian(uint32_t value, uint8_t* buffer) {
    buffer[3] = (value >> 24) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[0] = value & 0xFF;
}

uint16_t Conversion::littleEndianToUint16(const uint8_t* buffer) {
    return (static_cast<uint16_t>(buffer[1]) << 8) |
           static_cast<uint16_t>(buffer[0]);
}

uint32_t Conversion::littleEndianToUint32(const uint8_t* buffer) {
    return (static_cast<uint32_t>(buffer[3]) << 24) |
           (static_cast<uint32_t>(buffer[2]) << 16) |
           (static_cast<uint32_t>(buffer[1]) << 8)  |
           static_cast<uint32_t>(buffer[0]);
}

float Conversion::convertToFloat(const uint8_t* buffer) {
    return *((float *)buffer);
}
