#pragma once

#include "i2cbus.h"
#include <cstdint>
#include <string>

class MLXSensor
{
public:
    static constexpr uint8_t DefaultAddress = 0x5A;

    struct Readout
    {
        double ambientTemperatureC = 0.0;
        double objectTemperatureC = 0.0;
        double skyTemperatureC = 0.0;
        double skyTemperatureDiffC = 0.0;
        bool valid = false;
    };

    explicit MLXSensor(uint8_t deviceAddress = DefaultAddress);
    ~MLXSensor();

    bool open(int busNumber = 1);
    void close();
    bool isOpen() const;

    Readout read(double ambientReferenceC = 0.0) const;
    std::string lastError() const;

private:
    bool readWord(uint8_t reg, uint16_t &value) const;
    static double rawToCelsius(uint16_t raw);

private:
    mutable I2CBus m_Bus;
};