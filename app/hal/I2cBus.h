#pragma once
#include "main.h"
#include <cstdint>

namespace hal {

// Wraps I2C_HandleTypeDef.  All address parameters are 7-bit (right-aligned);
// the wrapper shifts left by 1 before passing to HAL.
class I2cBus
{
public:
    explicit I2cBus(I2C_HandleTypeDef* hi2c) : _hi2c(hi2c) {}

    bool write(uint8_t addr7, const uint8_t* data, uint16_t len,
               uint32_t timeoutMs = 20) const
    {
        return HAL_I2C_Master_Transmit(_hi2c,
                                       static_cast<uint16_t>(addr7 << 1),
                                       const_cast<uint8_t*>(data), len,
                                       timeoutMs) == HAL_OK;
    }

    bool read(uint8_t addr7, uint8_t* data, uint16_t len,
              uint32_t timeoutMs = 20) const
    {
        return HAL_I2C_Master_Receive(_hi2c,
                                      static_cast<uint16_t>(addr7 << 1),
                                      data, len,
                                      timeoutMs) == HAL_OK;
    }

    // Register-addressed write (e.g. EEPROM, sensor config registers)
    bool memWrite(uint8_t addr7, uint16_t memAddr, uint16_t memAddrSize,
                  const uint8_t* data, uint16_t len, uint32_t timeoutMs = 20) const
    {
        return HAL_I2C_Mem_Write(_hi2c,
                                  static_cast<uint16_t>(addr7 << 1),
                                  memAddr, memAddrSize,
                                  const_cast<uint8_t*>(data), len,
                                  timeoutMs) == HAL_OK;
    }

    bool memRead(uint8_t addr7, uint16_t memAddr, uint16_t memAddrSize,
                 uint8_t* data, uint16_t len, uint32_t timeoutMs = 20) const
    {
        return HAL_I2C_Mem_Read(_hi2c,
                                 static_cast<uint16_t>(addr7 << 1),
                                 memAddr, memAddrSize,
                                 data, len,
                                 timeoutMs) == HAL_OK;
    }

    bool isDeviceReady(uint8_t addr7, uint32_t trials = 3,
                       uint32_t timeoutMs = 20) const
    {
        return HAL_I2C_IsDeviceReady(_hi2c,
                                      static_cast<uint16_t>(addr7 << 1),
                                      trials, timeoutMs) == HAL_OK;
    }

    I2C_HandleTypeDef* handle() const { return _hi2c; }

private:
    I2C_HandleTypeDef* _hi2c;
};

} // namespace hal
