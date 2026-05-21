#pragma once
#include "hal/GpioPin.h"
#include <cstdint>

namespace drv {

// TPS1663 smart high-side switch.
// EN is active-high; FAULT is open-drain active-low (use internal pull-up).
// Current sensing via an external ADC channel is optional — read from outside
// this class and pass to PowerService for reporting.
class TPS1663
{
public:
    struct Pins
    {
        hal::GpioPin enable; // output, active-high
        hal::GpioPin fault;  // input,  active-low (open-drain on TPS1663)
    };

    TPS1663(Pins pins, uint8_t channelId);

    void on();
    void off();

    bool isOn()      const { return _on; }
    bool hasFault()  const { return !_pins.fault.read(); } // active-low
    uint8_t id()     const { return _id; }

private:
    Pins    _pins;
    uint8_t _id;
    bool    _on = false;
};

} // namespace drv
