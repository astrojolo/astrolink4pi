#pragma once
#include "hal/I2cBus.h"
#include <cstdint>
#include <array>

namespace drv {

// MLX90641 — 16×12 far-infrared thermal sensor array (192 pixels).
// EEPROM calibration is read once during init() and stored in _params.
class MLX90641
{
public:
    static constexpr uint8_t  DEFAULT_ADDR = 0x33;
    static constexpr uint16_t PIXEL_COUNT  = 192; // 16 × 12

    explicit MLX90641(hal::I2cBus& bus, uint8_t addr = DEFAULT_ADDR);

    // Reads EEPROM calibration data; must be called before getFrame().
    bool init();

    // Triggers a measurement and fills frame[] with temperature values in °C.
    // frame must point to at least PIXEL_COUNT floats.
    bool getFrame(float* frame);

    float ambientTemp() const { return _ta; } // Ta from last frame

private:
    bool readEeprom();
    bool extractParams();
    bool readRawFrame(uint16_t* raw);
    void calculateFrame(const uint16_t* raw, float* frame);

    hal::I2cBus& _bus;
    uint8_t      _addr;
    float        _ta = 0.0f;

    // Calibration coefficients (from MLX90641 application note)
    struct CalibParams
    {
        float kVdd, vdd25, kvPtat, ktPtat, vPtat25, alphaPTAT;
        float gainEE;
        float tgc;
        float cpKv, cpKta, cpAlpha[2], cpOffset[2];
        float kSTo1, kSTo2, ct[4];
        float alpha[PIXEL_COUNT];
        int16_t offset[PIXEL_COUNT];
        float kta[PIXEL_COUNT];
        float kv[PIXEL_COUNT];
        float ilChessC[3];
    } _params = {};

    bool _calibrated = false;

    static constexpr uint16_t EEPROM_ADDR = 0x2400;
    static constexpr uint16_t EEPROM_SIZE = 832;
    static constexpr uint16_t RAM_ADDR    = 0x0400;
};

} // namespace drv
