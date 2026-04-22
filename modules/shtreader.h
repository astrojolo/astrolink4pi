#pragma once

#include <cstdint>
#include <string>

#include "basecomponent.h"

class SHTReader : public BaseComponent
{
public:
    struct Readings
    {
        double temperature = 0.0;
        double humidity    = 0.0;
        double dewPoint    = 0.0;
    };

    SHTReader(const std::string &deviceName);
    ~SHTReader();

    bool open();
    void close();
    bool isOpen() const;
    bool read(Readings &out, int mode);
    bool ensureOpen();
    bool startMeasurement();
    bool readMeasurement(Readings &out);
    uint8_t crc8(const uint8_t *data, size_t len) const;
    
private:
    int m_Fd = -1;
    uint8_t m_ShtAddress = 0x44;
    Readings m_LastReadings;
};