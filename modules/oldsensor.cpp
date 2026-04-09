#include "oldsensor.h"

#include <array>
#include <cmath>

OldSensor::OldSensor(const Config &config)
    : m_Bus(config.deviceAddress)
    , m_Config(config)
{
}

OldSensor::~OldSensor() = default;

bool OldSensor::open(int busNumber)
{
    return m_Bus.open(busNumber);
}

void OldSensor::close()
{
    m_Bus.close();
}

bool OldSensor::isOpen() const
{
    return m_Bus.isOpen();
}

std::string OldSensor::lastError() const
{
    return m_Bus.getLastError();
}

OldSensor::Readout OldSensor::read() const
{
    Readout result;

    if (!m_Bus.isOpen())
        return result;

    const char cmd = static_cast<char>(m_Config.triggerCommand);
    if (m_Bus.write(&cmd, 1) != 1)
        return result;

    usleep(180000);

    std::array<char, 2> buf{};
    if (m_Bus.read(buf.data(), buf.size()) != static_cast<int>(buf.size()))
        return result;

    result.raw = static_cast<uint16_t>(
        (static_cast<uint8_t>(buf[0]) << 8) |
         static_cast<uint8_t>(buf[1]));

    // BH1750-style przeliczenie; jeśli stary sensor jest inny,
    // tu wystarczy podmienić tę jedną linijkę.
    result.lux = static_cast<double>(result.raw) / 1.2;
    result.sqm = calculateSqm(result.lux, m_Config.filterCoeff, m_Config.sqmOffset);
    result.valid = true;
    return result;
}

double OldSensor::calculateSqm(double lux, double filterCoeff, double offset)
{
    if (lux <= 0.0)
        return 0.0;

    return 12.6 - 2.5 * std::log10(lux) + filterCoeff + offset;
}