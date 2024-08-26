#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <cstdint>

class Conversion {
public:
    static void uint16_to_big_endian(uint16_t value, uint8_t* buffer);

    static uint16_t big_endian_to_uint16(const uint8_t* buffer);

    static void uint32_to_big_endian(uint32_t value, uint8_t* buffer);

    static uint32_t big_endian_to_uint32(const uint8_t* buffer);

    static void uint16_to_little_endian(uint16_t value, uint8_t* buffer);

    static uint16_t little_endian_to_uint16(const uint8_t* buffer);

    static void uint32_to_little_endian(uint32_t value, uint8_t* buffer);

    static uint32_t little_endian_to_uint32(const uint8_t* buffer);

    static float convert_to_float(const uint8_t* buffer);
};

#endif // CONVERSION_HPP
