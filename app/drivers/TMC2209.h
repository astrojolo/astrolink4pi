#pragma once
#include "hal/UartBus.h"
#include "hal/GpioPin.h"
#include <cstdint>

namespace drv {

// TMC2209 stepper driver — UART configuration + STEP/DIR/EN motion control.
//
// Wire:   STM32 UART TX looped to RX via 1 kΩ → TMC2209 PDN_UART pin
//         STEP, DIR, EN as push-pull outputs
//
// Usage:
//   drv::TMC2209::Config cfg { uartBus, stepPin, dirPin, enablePin, 0, 1e-5f };
//   drv::TMC2209 motor(cfg);
//   motor.init();
//   motor.setMaxVelocity(5000.f);
//   motor.enable();
//   motor.moveTo(10000);
//   // In timer ISR:  motor.tick();

class TMC2209
{
public:
    // ── UART register addresses ──────────────────────────────────────────────
    struct Reg
    {
        static constexpr uint8_t GCONF      = 0x00;
        static constexpr uint8_t GSTAT      = 0x01;
        static constexpr uint8_t IFCNT      = 0x02;
        static constexpr uint8_t IHOLD_IRUN = 0x10;
        static constexpr uint8_t TPOWERDOWN = 0x11;
        static constexpr uint8_t TSTEP      = 0x12;
        static constexpr uint8_t TPWMTHRS   = 0x13;
        static constexpr uint8_t CHOPCONF   = 0x6C;
        static constexpr uint8_t DRVSTATUS  = 0x6F;
        static constexpr uint8_t PWMCONF    = 0x70;
        static constexpr uint8_t PWM_SCALE  = 0x71;
    };

    // ── Hardware binding ─────────────────────────────────────────────────────
    struct Config
    {
        hal::UartBus& uart;
        hal::GpioPin  stepPin;
        hal::GpioPin  dirPin;
        hal::GpioPin  enablePin;        // active-low
        uint8_t       address;          // UART slave address 0–3 (MS1/MS2 pins)
        float         tickInterval;     // seconds per tick() call, e.g. 1e-5f
        uint8_t       stepPulseNops = 50; // NOP count for STEP high time (≥100 ns)
    };

    explicit TMC2209(Config& config);

    // Initialises GPIO state and writes default registers.
    // Returns false if the UART write fails (UART wiring fault).
    bool init();

    // ── Enable / Disable ─────────────────────────────────────────────────────
    void enable();
    void disable();
    bool isEnabled() const { return _enabled; }

    // ── Current control ──────────────────────────────────────────────────────
    // irun / ihold: 0–31 (0 = ~3%, 31 = 100% of maximum RMS current)
    bool setCurrentScaling(uint8_t irun, uint8_t ihold, uint8_t iholdDelay = 5);
    bool setRunCurrent(uint8_t irun);
    bool setHoldCurrent(uint8_t ihold);

    // ── Chopper / mode config ────────────────────────────────────────────────
    // microsteps: 1, 2, 4, 8, 16, 32, 64, 128, 256
    bool setMicrosteps(uint16_t microsteps);
    // false = StealthChop (quiet), true = SpreadCycle (higher speed)
    bool setSpreadCycle(bool enable);
    bool setDirectionInverted(bool invert);
    bool setStealthChopThreshold(uint32_t tstep);

    // ── Position ─────────────────────────────────────────────────────────────
    int32_t getPosition() const       { return _position; }
    void    setPosition(int32_t pos)  { _position = pos;  }

    // ── Motion parameters ────────────────────────────────────────────────────
    void  setMaxVelocity(float stepsPerSec)   { _maxVelocity  = stepsPerSec;  }
    void  setAcceleration(float stepsPerSec2) { _acceleration = stepsPerSec2; }
    float getMaxVelocity()  const             { return _maxVelocity;  }
    float getAcceleration() const             { return _acceleration; }

    // ── Motion commands ──────────────────────────────────────────────────────
    void moveTo(int32_t absoluteSteps);
    void moveBy(int32_t relativeSteps);
    void stop();
    bool    isRunning() const { return _state == State::MOVING; }
    int32_t getTarget()  const { return _target; }

    // ── Low-level UART register access ───────────────────────────────────────
    bool writeRegister(uint8_t reg, uint32_t value);
    bool readRegister(uint8_t reg, uint32_t& value);

    // ── Status ───────────────────────────────────────────────────────────────
    uint32_t getDriverStatus();
    bool     isStalled();
    bool     isOvertemp();

    // ── ISR entry point ──────────────────────────────────────────────────────
    // Call from a hardware timer ISR at exactly 1 / tickInterval Hz.
    void tick();

private:
    uint8_t calcCRC(const uint8_t* data, uint8_t len) const;
    void    stepPulse();

    enum class State { IDLE, MOVING };

    Config&  _config;
    State    _state        = State::IDLE;
    bool     _enabled      = false;
    int8_t   _direction    = 1;
    int32_t  _position     = 0;
    int32_t  _target       = 0;
    float    _velocity     = 0.0f;
    float    _maxVelocity  = 1000.0f;
    float    _acceleration = 500.0f;
    float    _accumulator  = 0.0f;

    uint32_t _gconf      = 0;
    uint32_t _chopconf   = 0;
    uint8_t  _irun       = 16;
    uint8_t  _ihold      = 8;
    uint8_t  _iholdDelay = 5;

    static constexpr float    MIN_VELOCITY  = 10.0f;
    static constexpr uint32_t UART_TIMEOUT  = 10u;
    static constexpr uint8_t  SYNC_BYTE     = 0x05u;
};

} // namespace drv
