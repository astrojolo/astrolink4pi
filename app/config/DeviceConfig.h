#pragma once
#include "services/MotorService.h"
#include <cstdint>
#include <array>

namespace cfg {

// All user-configurable parameters persisted in EEPROM.
// Add new fields at the END to preserve backwards compatibility.
struct DeviceConfig
{
    // ── Magic / version ───────────────────────────────────────────────────────
    uint32_t magic   = 0xA57EC0DEu; // validity sentinel
    uint8_t  version = 1;

    // ── Motor ─────────────────────────────────────────────────────────────────
    int32_t  motorPosition    = 0;       // last known position (saved on stop)
    float    motorMaxVelocity = 5000.0f;
    float    motorAccel       = 2000.0f;
    uint16_t motorMicrosteps  = 16;
    uint8_t  motorRunCurrent  = 20;      // 0–31
    uint8_t  motorHoldCurrent = 8;
    int32_t  motorMinPos      = -100000;
    int32_t  motorMaxPos      =  100000;
    bool     motorInverted    = false;

    // ── PWM outputs ───────────────────────────────────────────────────────────
    uint8_t  pwmDuty[2] = {0, 0};        // 0–100 %, restored at boot

    // ── DC outputs ────────────────────────────────────────────────────────────
    bool     dcOnAtBoot[3] = {false, false, false};

    // ── Sensor intervals ─────────────────────────────────────────────────────
    uint16_t envPollMs   = 5000;   // SHT30 poll period in ms
    uint16_t luxPollMs   = 2000;   // TSL2561
    uint16_t powerPollMs = 1000;   // INA219
    uint16_t irPollMs    = 500;    // MLX90641

    // ── Padding / reserved for future fields ─────────────────────────────────
    uint8_t  _reserved[16] = {};
};

} // namespace cfg
