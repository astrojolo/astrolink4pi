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

    SHTReader(uint8_t shtAddress, const std::string &deviceName);
    ~SHTReader();

    bool open();
    void close();
    bool isOpen() const;
    bool read(Readings &out);

private:
    int m_Fd = -1;
    uint8_t m_ShtAddress = 0x44;
    uint8_t readIndex = 0;
    Readings m_LastReadings;
};