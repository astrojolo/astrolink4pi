#pragma once
#include "hal/I2cBus.h"
#include <cstdint>

namespace drv {

// AT24Cxx I2C EEPROM driver.  Handles page-boundary write splitting.
// Typical parts: AT24C02 (256 B), AT24C32 (4 kB), AT24C256 (32 kB).
class AT24Cxx
{
public:
    // capacityBytes: total capacity, e.g. 256, 4096, 32768
    // pageBytes:     write page size from datasheet
    AT24Cxx(hal::I2cBus& bus, uint8_t addr7, uint32_t capacityBytes,
            uint8_t pageBytes = 32);

    bool writeByte(uint16_t address, uint8_t data);
    bool readByte(uint16_t address, uint8_t& data);

    // Writes up to len bytes, automatically splitting across page boundaries.
    // Includes the required inter-page write cycle delay (~5 ms).
    bool write(uint16_t address, const uint8_t* data, uint16_t len);
    bool read(uint16_t address, uint8_t* data, uint16_t len);

    // Convenience helpers for plain-old-data structs
    template<typename T>
    bool writeObject(uint16_t address, const T& obj)
    {
        return write(address,
                     reinterpret_cast<const uint8_t*>(&obj),
                     static_cast<uint16_t>(sizeof(T)));
    }

    template<typename T>
    bool readObject(uint16_t address, T& obj)
    {
        return read(address,
                    reinterpret_cast<uint8_t*>(&obj),
                    static_cast<uint16_t>(sizeof(T)));
    }

    uint32_t capacity() const { return _capacity; }

private:
    hal::I2cBus& _bus;
    uint8_t      _addr;
    uint32_t     _capacity;
    uint8_t      _pageSize;
};

} // namespace drv
