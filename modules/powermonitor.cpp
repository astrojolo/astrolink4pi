#include "powermonitor.h"

#include <wiringPiI2C.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{
uint64_t nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
}

PowerMonitor::PowerMonitor(uint8_t adsAddress, uint8_t acsType)
    : m_AdsAddress(adsAddress), m_AcsType(acsType)
{
}

PowerMonitor::~PowerMonitor()
{
    close();
}

bool PowerMonitor::open(int bus)
{
    close();

    m_Fd = wiringPiI2CSetupInterface(busPath(bus).c_str(), m_AdsAddress);
    m_HaveEnergyBaseline = false;
    m_LastSampleMs = 0;
    m_Ah = 0.0;
    m_Wh = 0.0;

    return m_Fd >= 0;
}

void PowerMonitor::close()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;
    }

    m_HaveEnergyBaseline = false;
    m_LastSampleMs = 0;
}

bool PowerMonitor::isOpen() const
{
    return m_Fd >= 0;
}

bool PowerMonitor::read(PowerMonitor::Readings &out)
{
    if (!isOpen())
        return false;

    double vin = 0.0;
    double vreg = 0.0;
    double acsVoltage = 0.0;

    if (!readChannel(Channel::VIN, vin))
        return false;

    if (!readChannel(Channel::VREG, vreg))
        return false;

    if (!readChannel(Channel::ACS, acsVoltage))
        return false;

    const double current = adcToCurrent(acsVoltage);
    const double power = vin * current;

    const uint64_t now = nowMs();

    if (m_HaveEnergyBaseline && now > m_LastSampleMs)
    {
        const double dtHours = static_cast<double>(now - m_LastSampleMs) / 3600000.0;
        m_Ah += current * dtHours;
        m_Wh += power * dtHours;
    }

    m_LastSampleMs = now;
    m_HaveEnergyBaseline = true;

    out.vin = vin;
    out.vreg = vreg;
    out.current = current;
    out.power = power;
    out.ah = m_Ah;
    out.wh = m_Wh;

    return true;
}

bool PowerMonitor::readChannel(Channel channel, double &voltage)
{
    if (!isOpen())
        return false;

    uint16_t muxBits = 0;

    switch (channel)
    {
        case Channel::VIN:
            muxBits = 0x4000; // AIN0 vs GND
            break;
        case Channel::VREG:
            muxBits = 0x5000; // AIN1 vs GND
            break;
        case Channel::ACS:
            muxBits = 0x6000; // AIN2 vs GND
            break;
    }

    const uint16_t config = static_cast<uint16_t>(0x8000 | muxBits | 0x0200 | ADS_BASE_CONFIG);

    if (!writeRegister16(REG_CONFIG, config))
        return false;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint16_t rawReg = 0;
    if (!readRegister16(REG_CONVERSION, rawReg))
        return false;

    const int16_t raw = static_cast<int16_t>(rawReg);
    voltage = adcToVoltage(raw);
    return true;
}

bool PowerMonitor::writeRegister16(uint8_t reg, uint16_t value)
{
    const int rc = wiringPiI2CWriteReg16(m_Fd, reg, static_cast<int>(swap16(value)));
    return rc >= 0;
}

bool PowerMonitor::readRegister16(uint8_t reg, uint16_t &value)
{
    const int rc = wiringPiI2CReadReg16(m_Fd, reg);
    if (rc < 0)
        return false;

    value = swap16(static_cast<uint16_t>(rc));
    return true;
}

uint16_t PowerMonitor::swap16(uint16_t value)
{
    return static_cast<uint16_t>((value >> 8) | (value << 8));
}

double PowerMonitor::adcToVoltage(int16_t raw) const
{
    // ADS1115, full-scale ±4.096V => 125 uV / LSB
    return static_cast<double>(raw) * 0.000125;
}

double PowerMonitor::adcToCurrent(double sensorVoltage) const
{
    // Typowy ACS7xx: środek w okolicy Vcc/2.
    // Tu zakładam tor analogowy 0..3.3V z offsetem ~1.65V.
    constexpr double zeroCurrentOffset = 1.65;

    const double sensitivity =
        (m_AcsType == 1)
            ? 0.185   // 5A
            : 0.100;  // 20A

    double current = (sensorVoltage - zeroCurrentOffset) / sensitivity;

    if (std::fabs(current) < 0.03)
        current = 0.0;

    return current;
}

std::string PowerMonitor::busPath(int bus) const
{
    return "/dev/i2c-" + std::to_string(bus);
}