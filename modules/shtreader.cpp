#include "shtreader.h"

#include <cerrno>
#include <cmath>
#include <cstring>

#include <wiringPi.h>
#include <wiringPiI2C.h>

SHTReader::SHTReader(const std::string &deviceName)
    : BaseComponent(deviceName, "SHTReader")
{
}

SHTReader::~SHTReader()
{
    close();
}

bool SHTReader::open()
{
    close();

    m_Fd = wiringPiI2CSetup(m_ShtAddress);
    if (m_Fd < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "SHT ID %d failed: errno=%d (%s)", m_Fd, errno, std::strerror(errno));

        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "SHT I2C setup failed: errno=%d (%s)",
                     errno, std::strerror(errno));
        return false;
    }
    else
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "SHT ID %d OK: errno=%d (%s)", m_Fd, errno, std::strerror(errno));

    }

    return m_Fd >= 0;
}

void SHTReader::close()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;
    }
}

bool SHTReader::isOpen() const
{
    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "SHT ID %d is open?", m_Fd);
    return m_Fd >= 0;
}

bool SHTReader::read(SHTReader::Readings &out, int mode)
{
    if (!isOpen())
    {
        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "I2C not available - read error.");
        return false;
    }

    out = m_LastReadings;

    if ((mode % 2) == 0)
    {
        uint8_t cmd[2] = {0x24, 0x00};

        if (wiringPiI2CRawWrite(m_Fd, cmd, 2) != 2)
        {
            int err = errno;
            DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                         "SHT raw write failed: fd=%d errno=%d (%s)",
                         m_Fd, err, std::strerror(err));
            return false;
        }
    }
    else
    {
        uint8_t buf[6] = {0};

        if (wiringPiI2CRawRead(m_Fd, buf, 6) != 6)
        {
            int err = errno;
            DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                         "SHT raw read failed: fd=%d errno=%d (%s)",
                         m_Fd, err, std::strerror(err));
            return false;
        }

        const uint16_t rawTemp = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
        const uint16_t rawHumidity = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];

        const double cTemp = -45.0 + 175.0 * static_cast<double>(rawTemp) / 65535.0;
        double humidity = 100.0 * static_cast<double>(rawHumidity) / 65535.0;

        if (humidity < 0.0)
            humidity = 0.0;
        if (humidity > 100.0)
            humidity = 100.0;

        double dewPoint = cTemp;

        if (humidity > 0.0)
        {
            const double a = 17.271;
            const double b = 237.7;
            const double tempAux = (a * cTemp) / (b + cTemp) + std::log(humidity * 0.01);
            dewPoint = (b * tempAux) / (a - tempAux);
        }

        out.temperature = cTemp;
        out.humidity = humidity;
        out.dewPoint = dewPoint;

        m_LastReadings = out;
    }

    return true;
}