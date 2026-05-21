#pragma once
#include "main.h"
#include <cstdint>

namespace hal {

// Value-type wrapper around a single GPIO pin.
// Holds port + pin mask; zero runtime overhead — inlines to HAL calls.
class GpioPin
{
public:
    GpioPin() = default;
    GpioPin(GPIO_TypeDef* port, uint16_t pin) : _port(port), _pin(pin) {}

    void set()              { HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);   }
    void reset()            { HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET); }
    void write(bool high)   { HAL_GPIO_WritePin(_port, _pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET); }
    void toggle()           { HAL_GPIO_TogglePin(_port, _pin); }
    bool read() const       { return HAL_GPIO_ReadPin(_port, _pin) == GPIO_PIN_SET; }

    bool isValid() const    { return _port != nullptr; }

private:
    GPIO_TypeDef* _port = nullptr;
    uint16_t      _pin  = 0;
};

} // namespace hal
