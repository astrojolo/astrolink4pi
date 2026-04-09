#include "shtsensor.h"

#include <array>
#include <cmath>
#include <unistd.h>

namespace
{
constexpr uint8_t SHT_CMD_MEAS_HIGHREP_STRETCH_OFF_MSB = 0x24;
constexpr uint8_t SHT_CMD_MEAS_HIGHREP_STRETCH_OFF_LSB = 0x00;
}

SHTSensor::SHTSensor(uint8_t deviceAddress)
    : m_Bus(deviceAddress)
{
}

SHTSensor::~SHTSensor() = default;

bool SHTSensor::open(int busNumber)
{
    return m_Bus.open(busNumber);
}

void SHTSensor::close()
{
    m_Bus.close();
}

bool SHTSensor::isOpen() const
{
    return m_Bus.isOpen();
}

std::string SHTSensor::lastError() const
{
    return m_Bus.getLastError();
}

SHTSensor::Readout SHTSensor::read() const
{
    Readout result;

    if (!m_Bus.isOpen())
        return result;

    const std::array<char, 2> cmd = {
        static_cast<char>(SHT_CMD_MEAS_HIGHREP_STRETCH_OFF_MSB),
        static_cast<char>(SHT_CMD_MEAS_HIGHREP_STRETCH_OFF_LSB)
    };

    if (m_Bus.write(cmd.data(), cmd.size()) != static_cast<int>(cmd.size()))
        return result;

    // typowo 15 ms wystarcza dla high repeatability
    usleep(20000);

    std::array<char, 6> raw{};
    if (m_Bus.read(raw.data(), raw.size()) != static_cast<int>(raw.size()))
        return result;

    const auto *data = reinterpret_cast<const uint8_t *>(raw.data());

    if (!checkCrc(data, 2, data[2]) || !checkCrc(data + 3, 2, data[5]))
        return result;

    const uint16_t rawTemp = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    const uint16_t rawHum  = (static_cast<uint16_t>(data[3]) << 8) | data[4];

    result.temperatureC = -45.0 + 175.0 * static_cast<double>(rawTemp) / 65535.0;
    result.humidityRH   = 100.0 * static_cast<double>(rawHum) / 65535.0;

    if (result.humidityRH < 0.0)
        result.humidityRH = 0.0;
    if (result.humidityRH > 100.0)
        result.humidityRH = 100.0;

    result.dewPointC = calculateDewPoint(result.temperatureC, result.humidityRH);
    result.valid = true;
    return result;
}

bool SHTSensor::checkCrc(const uint8_t *data, int len, uint8_t crc)
{
    uint8_t calc = 0xFF;

    for (int i = 0; i < len; ++i)
    {
        calc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (calc & 0x80)
                calc = static_cast<uint8_t>((calc << 1) ^ 0x31);
            else
                calc <<= 1;
        }
    }

    return calc == crc;
}

double SHTSensor::calculateDewPoint(double temperatureC, double humidityRH)
{
    if (humidityRH <= 0.0)
        return temperatureC;

    constexpr double a = 17.62;
    constexpr double b = 243.12;

    const double gamma = std::log(humidityRH / 100.0) + (a * temperatureC) / (b + temperatureC);
    return (b * gamma) / (a - gamma);
}