#pragma once

#include <cstdint>
#include <string>

#include "basecomponent.h"

class MLXReader : public BaseComponent
{
public:
    struct Readings
    {
        double ambientTemperature = 0.0; // Ta
        double objectTemperature  = 0.0; // To
        double tempDifference  = 0.0; // Td
    };

    MLXReader(const std::string &deviceName);
    ~MLXReader();

    bool open();
    void close();
    bool isOpen() const;

    bool read(Readings &out);

private:
    bool ensureOpen();
    void resetState();
    bool readWord(uint8_t reg, uint16_t &value);

private:
    int m_Fd = -1;
    uint8_t m_MlxAddress = 0x5A;
    Readings m_LastReadings;
};