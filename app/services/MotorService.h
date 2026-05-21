#pragma once
#include "drivers/TMC2209.h"
#include <cstdint>

namespace svc {

// Focuser motor service — wraps TMC2209 with position limits, homing, and
// safe parameter validation.  tick() is the ISR hot path; everything else
// is call-from-main-loop.
class MotorService
{
public:
    struct Config
    {
        int32_t minPosition  = -100000;
        int32_t maxPosition  =  100000;
        float   maxVelocity  =  5000.0f;  // steps/s
        float   acceleration =  2000.0f;  // steps/s²
        uint16_t microsteps  =  16;
        uint8_t  runCurrent  =  20;       // 0–31
        uint8_t  holdCurrent =   8;
    };

    explicit MotorService(drv::TMC2209& driver);

    bool init(const Config& cfg);

    // ── Commands (call from main loop / protocol handler) ─────────────────────
    bool moveTo(int32_t absoluteSteps);  // returns false if out of range
    bool moveBy(int32_t deltaSteps);
    void stop();   // decelerate to halt
    void abort();  // immediate disable

    bool setConfig(const Config& cfg);
    const Config& getConfig() const { return _cfg; }

    // ── State ─────────────────────────────────────────────────────────────────
    int32_t getPosition()     const;
    bool    isMoving()        const;
    bool    isEnabled()       const;
    bool    isStalled()       const;
    bool    isOvertemp()      const;

    void    syncPosition(int32_t pos); // update soft position without moving

    // ── ISR entry point (timer ISR, do not call from main loop) ──────────────
    void tick();

private:
    drv::TMC2209& _driver;
    Config        _cfg{};
};

} // namespace svc
