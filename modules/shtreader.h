#pragma once

#include "basecomponent.h"
#include <string>
#include <cstdint>
#include <cstddef>

class SHTReader : public BaseComponent
{
public:
    struct Readings
    {
        double temperature = 0.0;
        double humidity = 0.0;
        double dewPoint = 0.0;
    };

    explicit SHTReader(const std::string &deviceName);
    ~SHTReader() override;

    bool open();
    void close();
    bool isOpen() const;

    bool read(Readings &out);

private:
    enum class State
    {
        Idle,
        Measuring
    };

    bool ensureOpen();
    void resetState();

    bool startMeasurement();
    bool readMeasurement(Readings &out);

    uint8_t crc8(const uint8_t *data, size_t len) const;

private:
    int m_Fd = -1;
    int m_ShtAddress = 0x44; // albo Twoja wartość

    State m_State = State::Idle;
    uint32_t m_MeasurementStartMs = 0;

    Readings m_LastReadings{};
};