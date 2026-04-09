#include "tslsensor.h"

#include <array>
#include <cmath>

namespace
{
constexpr uint8_t CMD_BIT          = 0xA0;
constexpr uint8_t REG_ENABLE       = 0x00;
constexpr uint8_t REG_CONTROL      = 0x01;
constexpr uint8_t REG_CHAN0_LOW    = 0x14;
constexpr uint8_t REG_CHAN1_LOW    = 0x16;

constexpr uint8_t ENABLE_POWERON   = 0x01;
constexpr uint8_t ENABLE_POWEROFF  = 0x00;
constexpr uint8_t ENABLE_AEN       = 0x02;
}

TSLSensor::TSLSensor(uint8_t deviceAddress)
    : m_Bus(deviceAddress)
{
}

TSLSensor::~TSLSensor()
{
    if (m_Bus.isOpen())
    {
        m_Bus.writeRegisterByte(CMD_BIT | REG_ENABLE, ENABLE_POWEROFF);
    }
}

bool TSLSensor::open(int busNumber)
{
    return m_Bus.open(busNumber);
}

void TSLSensor::close()
{
    if (m_Bus.isOpen())
    {
        m_Bus.writeRegisterByte(CMD_BIT | REG_ENABLE, ENABLE_POWEROFF);
    }

    m_Bus.close();
    m_Initialized = false;
}

bool TSLSensor::isOpen() const
{
    return m_Bus.isOpen();
}

std::string TSLSensor::lastError() const
{
    return m_Bus.getLastError();
}

bool TSLSensor::initialize(const Config &config)
{
    if (!m_Bus.isOpen())
        return false;

    m_Config = config;

    if (m_Bus.writeRegisterByte(CMD_BIT | REG_ENABLE, ENABLE_POWERON | ENABLE_AEN) != 1)
        return false;

    if (m_Bus.writeRegisterByte(CMD_BIT | REG_CONTROL, m_Config.controlValue) != 1)
        return false;

    usleep(static_cast<useconds_t>(m_Config.integrationTimeMs * 1000));
    m_Initialized = true;
    return true;
}

TSLSensor::Readout TSLSensor::read() const
{
    Readout result;

    if (!m_Bus.isOpen() || !m_Initialized)
        return result;

    if (!readU16(CMD_BIT | REG_CHAN0_LOW, result.channel0))
        return result;

    if (!readU16(CMD_BIT | REG_CHAN1_LOW, result.channel1))
        return result;

    result.lux = calculateLux(result.channel0, result.channel1);
    result.sqm = calculateSqm(result.lux, m_Config.filterCoeff, m_Config.sqmOffset);
    result.valid = true;

    return result;
}

bool TSLSensor::readU16(uint8_t reg, uint16_t &value) const
{
    std::array<char, 2> buf{};
    if (m_Bus.readRegister(reg, buf.data(), buf.size()) != static_cast<int>(buf.size()))
        return false;

    const uint8_t lsb = static_cast<uint8_t>(buf[0]);
    const uint8_t msb = static_cast<uint8_t>(buf[1]);
    value = static_cast<uint16_t>((msb << 8) | lsb);
    return true;
}

double TSLSensor::calculateLux(uint16_t ch0, uint16_t ch1)
{
    if (ch0 == 0)
        return 0.0;

    const double ratio = static_cast<double>(ch1) / static_cast<double>(ch0);

    double lux = 0.0;
    if (ratio <= 0.5)
        lux = 0.0304 * ch0 - 0.062 * ch0 * std::pow(ratio, 1.4);
    else if (ratio <= 0.61)
        lux = 0.0224 * ch0 - 0.031 * ch1;
    else if (ratio <= 0.80)
        lux = 0.0128 * ch0 - 0.0153 * ch1;
    else if (ratio <= 1.30)
        lux = 0.00146 * ch0 - 0.00112 * ch1;
    else
        lux = 0.0;

    if (lux < 0.0)
        lux = 0.0;

    return lux;
}

double TSLSensor::calculateSqm(double lux, double filterCoeff, double offset)
{
    if (lux <= 0.0)
        return 0.0;

    // klasyczne przybliżenie lux -> mag/arcsec^2
    return 12.6 - 2.5 * std::log10(lux) + filterCoeff + offset;
}