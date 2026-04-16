#pragma once

#include "basecomponent.h"

#include <cstdint>
#include <string>

class TSLReader : public BaseComponent
{
public:
    struct Readings
    {
        double mpsas = 0.0;
        int full = 0;
        int ir = 0;
        int visible = 0;
        bool valid = false;
    };

    TSLReader(const std::string &deviceName);
    ~TSLReader();

    bool open();
    void close();
    bool isOpen() const;

    bool read(Readings &out);

    void setSQMOffset(double offset) { m_SQMOffset = offset; }

private:
    enum class TSLState
    {
        NotAvailable,
        Available,
        Initialized
    };

    static constexpr int TSL2591_ADC_TIME = 750; // ms
    static constexpr uint8_t TSL2591_COMMAND_BIT = 0xA0; // bits 7 and 5
    static constexpr uint8_t TSL2591_ENABLE_POWERON = 0x01;
    static constexpr uint8_t TSL2591_ENABLE_POWEROFF = 0x00;
    static constexpr uint8_t TSL2591_ENABLE_AEN = 0x02;
    static constexpr uint8_t TSL2591_ENABLE_AIEN = 0x10;
    static constexpr uint8_t TSL2591_REGISTER_ENABLE = 0x00;
    static constexpr uint8_t TSL2591_REGISTER_CONTROL = 0x01;
    static constexpr uint8_t TSL2591_REGISTER_CHAN0_LOW = 0x14;
    static constexpr uint8_t TSL2591_REGISTER_CHAN1_LOW = 0x16;

    bool probeSensor();
    bool initializeSensor();
    bool startIntegration();
    bool stopIntegration();
    bool readChannels(int &full, int &ir);

private:
    int m_Fd = -1;
    uint8_t m_TslAddress = 0x29;

    TSLState m_Mode = TSLState::NotAvailable;
    unsigned int m_AdcStartTime = 0;

    int m_NIter = 0;
    int m_FullCumulative = 0;
    int m_IrCumulative = 0;

    double m_SQMOffset = 0.0;
    double m_FilterCoeff = -1.2;

    Readings m_LastReadings;
};