#include "powermonitor.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <ads1115.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <unistd.h>

PowerMonitor::PowerMonitor(uint8_t adsAddress, uint8_t acsType, const std::string &deviceName)
    : BaseComponent(deviceName, "PowerMonitor"), m_AdsAddress(adsAddress), m_AcsType(acsType)
{
}

PowerMonitor::~PowerMonitor()
{
    close();
}

bool PowerMonitor::open()
{
    int wipi = wiringPiSetup();
    if (wipi < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "WiPi open failed: errno=%d (%s)", errno, std::strerror(errno));
        return 0;
    }

    m_Fd = wiringPiI2CSetup(m_AdsAddress);
    if (m_Fd < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "I2C setup failed: errno=%d (%s)", errno, std::strerror(errno));
        return 0;
    }

    return m_Fd >= 0;
}

void PowerMonitor::close()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;
    }
}

bool PowerMonitor::isOpen() const
{
    return m_Fd >= 0;
}

bool PowerMonitor::read(PowerMonitor::Readings &out)
{
    if (!isOpen())
        return false;

    uint8_t writeBuf[3];

    /*
    powerIndex 0-1 Vin WR, 2-3 Vreg WR, 4-5 Itot WR

    15 		- 1 	start single conv
    14:12	- 100 	Vin, 101 Vreg, 110 Itot, 111 Iref, 011 Ireal
    11:9  	- 001	+-4.096V
    8		- 1 single

    7:5		- 010 32SPS, 011 64SPS, 001 16SPS
    4:2		- 000 comparator
    1:0		- 11 comparator disable
    */

    writeBuf[0] = 0x01;
    writeBuf[1] = 0b11000011;
    writeBuf[2] = 0b00100011;
    out = m_LastReadings;

    if ((powerIndex % 2) == 0) // Trigger conversion
    {
        switch (powerIndex)
        {
        case 0:
            writeBuf[1] = 0b11000011;
            break;
        case 2:
            writeBuf[1] = 0b11010011;
            break;
        case 4:
            writeBuf[1] = 0b10110011;
            break;
        }
        uint16_t config = (writeBuf[1] << 8) | writeBuf[2];
        int written = wiringPiI2CWriteReg16(m_Fd, 0x01, __bswap_16(config));
        if (written < 0)
        {
            DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG, "ADS1115 write failed: errno=%d (%s)", errno, std::strerror(errno));
        }
    }
    else // Trigger read
    {
        uint8_t reg = 0x00;
        uint8_t buf[2] = {0, 0};

        if (wiringPiI2CRawWrite(m_Fd, &reg, 1) != 1)
        {
            int err = errno;
            DEBUGFDEVICE(getDeviceName().c_str(),
                         INDI::Logger::DBG_DEBUG,
                         "ADS1115 raw write(reg=0x00) failed: fd=%d errno=%d (%s)",
                         m_Fd, err, std::strerror(err));
            return false;
        }

        if (wiringPiI2CRawRead(m_Fd, buf, 2) != 2)
        {
            int err = errno;
            DEBUGFDEVICE(getDeviceName().c_str(),
                         INDI::Logger::DBG_DEBUG,
                         "ADS1115 raw read failed: fd=%d errno=%d (%s)",
                         m_Fd, err, std::strerror(err));
            return false;
        }

        int16_t raw = static_cast<int16_t>((buf[0] << 8) | buf[1]);

        if (raw < 0)
            raw = 0;

        switch (powerIndex)
        {
        case 1:
            out.vin = (float)raw / 32768.0 * 4.096 * 6.6;
            break;
        case 3:
            out.vreg = (float)raw / 32768.0 * 4.096 * 6.6;
            break;
        case 5:
            out.current = (float)raw / 32768.0 * 4.096 * 1 * ((m_AcsType == 0) ? 20 : 10.8);
            break;
        }
        out.power = out.vin * out.current;
        energyAs += out.current * 0.4;
        energyWs += out.vin * out.current * 0.4;
        out.ah = energyAs / 3600;
        out.wh = energyWs / 3600;

        m_LastReadings = out;
    }
    powerIndex++;
    if (powerIndex > 5)
        powerIndex = 0;

    return true;
}
