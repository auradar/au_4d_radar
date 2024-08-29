#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <cstdint>

class Conversion {
public:
    static void uint16ToBigEndian(uint16_t value, uint8_t* buffer);

    static uint16_t bigEndianToUint16(const uint8_t* buffer);

    static void uint32ToBigEndian(uint32_t value, uint8_t* buffer);

    static uint32_t bigEndianToUint32(const uint8_t* buffer);

    static void uint16ToLittleEndian(uint16_t value, uint8_t* buffer);

    static uint16_t littleEndianToUint16(const uint8_t* buffer);

    static void uint32ToLittleEndian(uint32_t value, uint8_t* buffer);

    static uint32_t littleEndianToUint32(const uint8_t* buffer);

    static float convertToFloat(const uint8_t* buffer);
};

#endif // CONVERSION_HPP
