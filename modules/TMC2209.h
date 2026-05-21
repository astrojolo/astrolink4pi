#pragma once

// Include the correct STM32 HAL header via your project's main.h,
// or replace with e.g. stm32f4xx_hal.h / stm32h7xx_hal.h.
#include "main.h"

#include <cstdint>
#include <cstdlib>

// TMC2209 stepper driver — UART configuration + STEP/DIR/EN motion control.
//
// Wire:   STM32 UART TX+RX tied together → TMC2209 PDN_UART pin (with 1kΩ series on TX)
//         STEP, DIR, EN as push-pull outputs
//
// Usage:
//   TMC2209::Config cfg { &huart2, STEP_GPIO_Port, STEP_Pin, DIR_GPIO_Port,
//                         DIR_Pin, EN_GPIO_Port, EN_Pin, /*address=*/0,
//                         /*tickInterval=*/1e-5f };   // 100 kHz timer
//   TMC2209 motor(cfg);
//   motor.init();
//   motor.setMaxVelocity(5000.f);    // steps/s
//   motor.setAcceleration(2000.f);   // steps/s²
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
        UART_HandleTypeDef *huart;        // UART handle (TX looped to RX for single-wire)
        GPIO_TypeDef       *stepPort;     // STEP GPIO port
        uint16_t            stepPin;      // STEP GPIO pin
        GPIO_TypeDef       *dirPort;      // DIR GPIO port
        uint16_t            dirPin;       // DIR GPIO pin
        GPIO_TypeDef       *enablePort;   // EN GPIO port  (EN is active-low)
        uint16_t            enablePin;    // EN GPIO pin
        uint8_t             address;      // UART slave address 0–3 (set via MS1/MS2)
        float               tickInterval; // Seconds per tick() call, e.g. 1e-5f for 100 kHz

        // NOP loop count for the STEP pulse high time.
        // 50 NOPs ≥ 100 ns on MCUs up to ~400 MHz.  Increase for faster cores.
        uint8_t stepPulseNops = 50;
    };

    explicit TMC2209(const Config &config);

    // Resets GPIO, writes initial registers.  Returns false on UART failure.
    bool init();

    // ── Enable / Disable ─────────────────────────────────────────────────────
    void enable();
    void disable();
    bool isEnabled() const { return _enabled; }

    // ── Current control ──────────────────────────────────────────────────────
    // irun / ihold : 0–31  (0 = ~3%, 31 = 100% of maximum RMS current)
    // iholdDelay   : 0–15  (motor power-down delay after standstill, × ~2 s / 16)
    bool setCurrentScaling(uint8_t irun, uint8_t ihold, uint8_t iholdDelay = 5);
    bool setRunCurrent(uint8_t irun);
    bool setHoldCurrent(uint8_t ihold);

    // ── Chopper / mode config ────────────────────────────────────────────────
    // microsteps: 1, 2, 4, 8, 16, 32, 64, 128, 256
    bool setMicrosteps(uint16_t microsteps);

    // false = StealthChop (quiet, lower speed), true = SpreadCycle (higher speed)
    bool setSpreadCycle(bool enable);

    // Inverts the motor shaft direction via GCONF.shaft bit
    bool setDirectionInverted(bool invert);

    // StealthChop upper velocity threshold.  Above this TSTEP value the driver
    // switches to SpreadCycle automatically (0 = threshold disabled).
    bool setStealthChopThreshold(uint32_t tstep);

    // ── Position ─────────────────────────────────────────────────────────────
    int32_t getPosition() const          { return _position; }
    void    setPosition(int32_t pos)     { _position = pos;  }  // No motion

    // ── Motion parameters ────────────────────────────────────────────────────
    void setMaxVelocity(float stepsPerSec)   { _maxVelocity  = stepsPerSec;  }
    void setAcceleration(float stepsPerSec2) { _acceleration = stepsPerSec2; }
    float getMaxVelocity()   const           { return _maxVelocity;  }
    float getAcceleration()  const           { return _acceleration; }

    // ── Motion commands ──────────────────────────────────────────────────────
    void moveTo(int32_t absoluteSteps);
    void moveBy(int32_t relativeSteps);
    void stop();                             // Immediate halt
    bool    isRunning() const { return _state == State::MOVING; }
    int32_t getTarget()  const { return _target; }

    // ── Low-level UART register access ───────────────────────────────────────
    bool     writeRegister(uint8_t reg, uint32_t value);
    bool     readRegister(uint8_t reg, uint32_t &value);

    // ── Status ───────────────────────────────────────────────────────────────
    uint32_t getDriverStatus();   // Raw DRVSTATUS register
    bool     isStalled();         // True when SG_RESULT == 0 and motor is moving
    bool     isOvertemp();        // Overtemperature flag (DRVSTATUS bit 25)

    // ── ISR entry point ──────────────────────────────────────────────────────
    // Call from a hardware timer ISR at exactly 1 / tickInterval Hz.
    // Generates STEP pulses and executes the trapezoidal velocity profile.
    void tick();

private:
    uint8_t calcCRC(const uint8_t *data, uint8_t len) const;
    void    stepPulse();

    enum class State { IDLE, MOVING };

    Config   _config;
    State    _state       = State::IDLE;
    bool     _enabled     = false;
    int8_t   _direction   = 1;      // +1 or -1, updated per tick
    int32_t  _position    = 0;
    int32_t  _target      = 0;
    float    _velocity    = 0.0f;   // Current speed, always ≥ 0 (steps/s)
    float    _maxVelocity = 1000.0f;
    float    _acceleration= 500.0f;
    float    _accumulator = 0.0f;   // Fractional step accumulator

    // Cached shadow copies of writable registers
    uint32_t _gconf    = 0;
    uint32_t _chopconf = 0;
    uint8_t  _irun     = 16;
    uint8_t  _ihold    = 8;
    uint8_t  _iholdDelay = 5;

    // Minimum speed to guarantee the last step is always taken
    static constexpr float    MIN_VELOCITY = 10.0f;  // steps/s
    static constexpr uint32_t UART_TIMEOUT = 10u;    // ms
    static constexpr uint8_t  SYNC_BYTE    = 0x05u;
};
