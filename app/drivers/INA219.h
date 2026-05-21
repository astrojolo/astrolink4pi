#pragma once
#include "hal/I2cBus.h"
#include <cstdint>

namespace drv {

// INA219 — bidirectional current/power monitor over I2C.
class INA219
{
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x40; // A0=GND, A1=GND

    struct Config
    {
        float shuntOhms   = 0.1f;  // shunt resistor value
        float maxCurrentA = 3.2f;  // used to calculate current LSB
    };

    explicit INA219(hal::I2cBus& bus, uint8_t addr = DEFAULT_ADDR);

    bool init(const Config& cfg = {});

    // Triggers a conversion and reads all registers.
    bool read();

    float busVoltageV()  const { return _busV;   }
    float shuntVoltageV() const { return _shuntV; }
    float currentA()     const { return _current; }
    float powerW()       const { return _power;   }
    bool  isValid()      const { return _valid;   }

private:
    bool  writeReg(uint8_t reg, uint16_t val);
    bool  readReg(uint8_t reg, uint16_t& val);

    hal::I2cBus& _bus;
    uint8_t      _addr;
    float        _currentLsb = 0.0f;
    float        _busV    = 0.0f;
    float        _shuntV  = 0.0f;
    float        _current = 0.0f;
    float        _power   = 0.0f;
    bool         _valid   = false;

    static constexpr uint8_t REG_CONFIG     = 0x00;
    static constexpr uint8_t REG_SHUNT_V    = 0x01;
    static constexpr uint8_t REG_BUS_V      = 0x02;
    static constexpr uint8_t REG_POWER      = 0x03;
    static constexpr uint8_t REG_CURRENT    = 0x04;
    static constexpr uint8_t REG_CALIBRATION= 0x05;
};

} // namespace drv
