#pragma once
#include "drivers/TPS1663.h"
#include "hal/PwmOutput.h"
#include <array>
#include <cstdint>

namespace svc {

// Manages three TPS1663 switchable 12 V DC outputs and two 40 kHz PWM outputs.
class PowerService
{
public:
    static constexpr uint8_t DC_CHANNELS  = 3;
    static constexpr uint8_t PWM_CHANNELS = 2;

    PowerService(std::array<drv::TPS1663*, DC_CHANNELS>  dc,
                 std::array<hal::PwmOutput*, PWM_CHANNELS> pwm);

    void init();

    // ── DC switchable outputs (TPS1663) ───────────────────────────────────────
    void dcOn(uint8_t ch);
    void dcOff(uint8_t ch);
    bool dcIsOn(uint8_t ch)    const;
    bool dcHasFault(uint8_t ch) const;

    // Reads all FAULT pins; call periodically from main loop.
    // Returns bitmask of channels with active faults (bit 0 = ch 0).
    uint8_t checkFaults();

    // ── PWM outputs ───────────────────────────────────────────────────────────
    // duty: 0–100 %
    void    setPwmDuty(uint8_t ch, uint8_t dutyPercent);
    uint8_t getPwmDuty(uint8_t ch) const;

private:
    std::array<drv::TPS1663*,  DC_CHANNELS>  _dc;
    std::array<hal::PwmOutput*, PWM_CHANNELS> _pwm;
    std::array<uint8_t, PWM_CHANNELS>         _pwmDuty{};
};

} // namespace svc
