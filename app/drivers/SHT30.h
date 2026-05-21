#pragma once
#include "hal/I2cBus.h"
#include <cstdint>

namespace drv {

// SHT30 — humidity + temperature sensor, single-shot measurement mode.
class SHT30
{
public:
    static constexpr uint8_t ADDR_LOW  = 0x44; // ADDR pin → GND
    static constexpr uint8_t ADDR_HIGH = 0x45; // ADDR pin → VDD

    explicit SHT30(hal::I2cBus& bus, uint8_t addr = ADDR_LOW);

    bool init();

    // Trigger a single-shot measurement and wait for result (~15 ms).
    bool measure();

    float temperature() const { return _tempC;    }  // °C
    float humidity()    const { return _humidityRH; } // %RH
    bool  isValid()     const { return _valid;     }

    bool softReset();
    bool readStatus(uint16_t& status);

private:
    bool sendCommand(uint16_t cmd);
    bool readWords(uint8_t* buf, uint8_t wordCount);
    static float  decodeTemp(uint16_t raw);
    static float  decodeHumidity(uint16_t raw);
    static uint8_t crc8(const uint8_t* data, uint8_t len);

    hal::I2cBus& _bus;
    uint8_t      _addr;
    float        _tempC      = 0.0f;
    float        _humidityRH = 0.0f;
    bool         _valid      = false;
};

} // namespace drv
