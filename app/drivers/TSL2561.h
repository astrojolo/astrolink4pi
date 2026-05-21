#pragma once
#include "hal/I2cBus.h"
#include <cstdint>

namespace drv {

// TSL2561 — digital luminosity / lux sensor.
// Integrates visible + IR (CH0) and IR-only (CH1) channels.
class TSL2561
{
public:
    static constexpr uint8_t ADDR_GND = 0x29; // ADDR pin → GND
    static constexpr uint8_t ADDR_FLT = 0x39; // ADDR pin floating
    static constexpr uint8_t ADDR_VDD = 0x49; // ADDR pin → VDD

    enum class Gain     : uint8_t { X1 = 0x00, X16 = 0x10 };
    enum class IntegTime: uint8_t { MS_13 = 0x00, MS_101 = 0x01, MS_402 = 0x02 };

    explicit TSL2561(hal::I2cBus& bus, uint8_t addr = ADDR_FLT);

    bool init(Gain gain = Gain::X1, IntegTime it = IntegTime::MS_402);

    // Reads ADC channels and computes lux.  Call after integration time has elapsed.
    bool measure();

    float    lux()     const { return _lux;   }
    uint16_t ch0()     const { return _ch0;   } // broadband (visible + IR)
    uint16_t ch1()     const { return _ch1;   } // IR only
    bool     isValid() const { return _valid; }

    void     powerOn();
    void     powerOff();

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readWord(uint8_t reg, uint16_t& val);
    float calculateLux(uint16_t ch0, uint16_t ch1) const;

    hal::I2cBus& _bus;
    uint8_t      _addr;
    Gain         _gain     = Gain::X1;
    IntegTime    _integTime = IntegTime::MS_402;
    uint16_t     _ch0  = 0;
    uint16_t     _ch1  = 0;
    float        _lux  = 0.0f;
    bool         _valid = false;
};

} // namespace drv
