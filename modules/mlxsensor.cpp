#include "mlxsensor.h"

#include <array>
#include <unistd.h>

namespace
{
constexpr uint8_t MLX_REG_AMBIENT = 0x06;
constexpr uint8_t MLX_REG_OBJECT1 = 0x07;
}

MLXSensor::MLXSensor(uint8_t deviceAddress)
    : m_Bus(deviceAddress)
{
}

MLXSensor::~MLXSensor() = default;

bool MLXSensor::open(int busNumber)
{
    return m_Bus.open(busNumber);
}

void MLXSensor::close()
{
    m_Bus.close();
}

bool MLXSensor::isOpen() const
{
    return m_Bus.isOpen();
}

std::string MLXSensor::lastError() const
{
    return m_Bus.getLastError();
}

MLXSensor::Readout MLXSensor::read(double ambientReferenceC) const
{
    Readout result;

    if (!m_Bus.isOpen())
        return result;

    int rawAmbient = 0;
    int rawObject = 0;

    if (!readWord(MLX_REG_AMBIENT, rawAmbient))
        return result;

    if (!readWord(MLX_REG_OBJECT1, rawObject))
        return result;

    result.ambientTemperatureC = rawToCelsius(rawAmbient);
    result.objectTemperatureC = rawToCelsius(rawObject);
    result.skyTemperatureC = result.objectTemperatureC;

    const double reference = ambientReferenceC != 0.0 ? ambientReferenceC : result.ambientTemperatureC;
    // result.skyTemperatureDiffC = reference - result.skyTemperatureC;
    result.skyTemperatureDiffC = rawObject;
    result.objectTemperatureC = rawAmbient;

    result.valid = true;
    return result;
}

bool MLXSensor::readWord(uint8_t reg, uint16_t &value) const
{
    std::array<char, 3> buf{};
    if (m_Bus.readRegister(reg, buf.data(), buf.size()) != static_cast<int>(buf.size()))
        return false;

    const uint8_t lsb = static_cast<uint8_t>(buf[0]);
    const uint8_t msb = static_cast<uint8_t>(buf[1]);

    value = static_cast<uint16_t>((msb << 8) | lsb);
    return true;
}

double MLXSensor::rawToCelsius(uint16_t raw)
{
    return (static_cast<double>(raw) * 0.02) - 273.15;
}