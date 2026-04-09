#pragma once

#include "i2cbus.h"
#include <cstdint>
#include <string>

class SHTSensor
{
public:
    static constexpr uint8_t DefaultAddress = 0x44;

    struct Readout
    {
        double temperatureC = 0.0;
        double humidityRH = 0.0;
        double dewPointC = 0.0;
        bool valid = false;
    };

    explicit SHTSensor(uint8_t deviceAddress = DefaultAddress);
    ~SHTSensor();

    bool open(int busNumber = 1);
    void close();
    bool isOpen() const;

    Readout read() const;
    std::string lastError() const;

private:
    static bool checkCrc(const uint8_t *data, int len, uint8_t crc);
    static double calculateDewPoint(double temperatureC, double humidityRH);

private:
    mutable I2CBus m_Bus;
};