#pragma once

#include "i2cbus.h"
#include <cstdint>
#include <string>

class OldSensor
{
public:
    struct Config
    {
        uint8_t deviceAddress = 0x23;   // domyślnie BH1750-like / legacy sensor
        uint8_t triggerCommand = 0x10;  // continuous H-resolution / przykładowy tryb
        double sqmOffset = 0.0;
        double filterCoeff = 0.0;
    };

    struct Readout
    {
        uint16_t raw = 0;
        double lux = 0.0;
        double sqm = 0.0;
        bool valid = false;
    };

    explicit OldSensor(const Config &config = Config{});
    ~OldSensor();

    bool open(int busNumber = 1);
    void close();
    bool isOpen() const;

    Readout read() const;
    std::string lastError() const;

private:
    static double calculateSqm(double lux, double filterCoeff, double offset);

private:
    mutable I2CBus m_Bus;
    Config m_Config;
};