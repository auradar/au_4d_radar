#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <cstdint>

class Conversion {
public:
    static inline void uint16ToBigEndian(uint16_t value, uint8_t* buffer) {
        buffer[0] = (value >> 8) & 0xFF;
        buffer[1] = value & 0xFF;
    }

    static inline uint16_t bigEndianToUint16(const uint8_t* buffer) {
        return (static_cast<uint16_t>(buffer[0]) << 8) |
            static_cast<uint16_t>(buffer[1]);
    }

    static inline void uint32ToBigEndian(uint32_t value, uint8_t* buffer) {
        buffer[0] = (value >> 24) & 0xFF;
        buffer[1] = (value >> 16) & 0xFF;
        buffer[2] = (value >> 8) & 0xFF;
        buffer[3] = value & 0xFF;
    }

    static inline uint32_t bigEndianToUint32(const uint8_t* buffer) {
        return (static_cast<uint32_t>(buffer[0]) << 24) |
            (static_cast<uint32_t>(buffer[1]) << 16) |
            (static_cast<uint32_t>(buffer[2]) << 8)  |
            static_cast<uint32_t>(buffer[3]);
    }

    static inline void uint16ToLittleEndian(uint16_t value, uint8_t* buffer) {
        buffer[1] = (value >> 8) & 0xFF;
        buffer[0] = value & 0xFF;
    }

    static inline void uint32ToLittleEndian(uint32_t value, uint8_t* buffer) {
        buffer[3] = (value >> 24) & 0xFF;
        buffer[2] = (value >> 16) & 0xFF;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[0] = value & 0xFF;
    }

    static inline uint16_t littleEndianToUint16(const uint8_t* buffer) {
        return (static_cast<uint16_t>(buffer[1]) << 8) |
            static_cast<uint16_t>(buffer[0]);
    }

    static inline uint32_t littleEndianToUint32(const uint8_t* buffer) {
        return (static_cast<uint32_t>(buffer[3]) << 24) |
            (static_cast<uint32_t>(buffer[2]) << 16) |
            (static_cast<uint32_t>(buffer[1]) << 8)  |
            static_cast<uint32_t>(buffer[0]);
    }

    static inline float convertToFloat(const uint8_t* buffer) {
        return *((float *)buffer);
    }

private:
    static std::mutex bufferMutex;
};

#endif // CONVERSION_HPP
