#pragma once

#include "i2cbus.h"
#include <cstdint>
#include <string>

class TSLSensor
{
public:
    static constexpr uint8_t DefaultAddress = 0x29;

    struct Config
    {
        uint8_t controlValue = 0x01;
        int integrationTimeMs = 200;
        double sqmOffset = 0.0;
        double filterCoeff = -1.2;
    };

    struct Readout
    {
        uint16_t channel0 = 0;
        uint16_t channel1 = 0;
        double lux = 0.0;
        double sqm = 0.0;
        bool valid = false;
    };

    explicit TSLSensor(uint8_t deviceAddress = DefaultAddress);
    ~TSLSensor();

    bool open(int busNumber = 1);
    void close();
    bool isOpen() const;

    bool initialize();
    bool initialize(const Config &config);

    Readout read() const;
    std::string lastError() const;

private:
    bool readU16(uint8_t reg, uint16_t &value) const;
    static double calculateLux(uint16_t ch0, uint16_t ch1);
    static double calculateSqm(double lux, double filterCoeff, double offset);

private:
    mutable I2CBus m_Bus;
    Config m_Config;
    bool m_Initialized = false;
};