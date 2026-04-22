#include "shtreader.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <indilogger.h>

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
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "SHT I2C setup failed: errno=%d (%s)",
                     errno, std::strerror(errno));
        return false;
    }

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                 "SHT I2C opened: fd=%d addr=0x%02X", m_Fd, m_ShtAddress);

    return true;
}

void SHTReader::close()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;

        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                    "SHT I2C closed.");
    }
}

bool SHTReader::isOpen() const
{
    return m_Fd >= 0;
}

bool SHTReader::read(SHTReader::Readings &out, int mode)
{
    out = m_LastReadings;

    if ((mode % 2) == 0)
    {
        // Faza 1: start pomiaru
        return startMeasurement();
    }
    else
    {
        // Faza 2: odczyt poprzednio uruchomionego pomiaru
        return readMeasurement(out);
    }
    // if (!isOpen())
    // {
    //     DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "I2C not available - read error.");
    //     return false;
    // }

    // out = m_LastReadings;

    // if ((mode % 2) == 0)
    // {
    //     uint8_t cmd[2] = {0x24, 0x00};

    //     if (wiringPiI2CRawWrite(m_Fd, cmd, 2) != 2)
    //     {
    //         int err = errno;
    //         DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
    //                      "SHT raw write failed: fd=%d errno=%d (%s)",
    //                      m_Fd, err, std::strerror(err));
    //         return false;
    //     }
    // }
    // else
    // {
    //     uint8_t buf[6] = {0};

    //     if (wiringPiI2CRawRead(m_Fd, buf, 6) != 6)
    //     {
    //         int err = errno;
    //         DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
    //                      "SHT raw read failed: fd=%d errno=%d (%s)",
    //                      m_Fd, err, std::strerror(err));
    //         return false;
    //     }

    //     const uint16_t rawTemp = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    //     const uint16_t rawHumidity = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];

    //     const double cTemp = -45.0 + 175.0 * static_cast<double>(rawTemp) / 65535.0;
    //     double humidity = 100.0 * static_cast<double>(rawHumidity) / 65535.0;

    //     if (humidity < 0.0)
    //         humidity = 0.0;
    //     if (humidity > 100.0)
    //         humidity = 100.0;

    //     double dewPoint = cTemp;

    //     if (humidity > 0.0)
    //     {
    //         const double a = 17.271;
    //         const double b = 237.7;
    //         const double tempAux = (a * cTemp) / (b + cTemp) + std::log(humidity * 0.01);
    //         dewPoint = (b * tempAux) / (a - tempAux);
    //     }

    //     out.temperature = cTemp;
    //     out.humidity = humidity;
    //     out.dewPoint = dewPoint;

    //     m_LastReadings = out;
    // }

    // return true;
}

uint8_t SHTReader::crc8(const uint8_t *data, size_t len) const
{
    // SHT3x CRC-8:
    // polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
    // init: 0xFF
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x80)
                crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
            else
                crc = static_cast<uint8_t>(crc << 1);
        }
    }

    return crc;
}

bool SHTReader::ensureOpen()
{
    if (isOpen())
        return true;

    return open();
}

bool SHTReader::startMeasurement()
{
    // Single shot, high repeatability, clock stretching disabled
    const uint8_t cmd[2] = {0x24, 0x00};

    if (!ensureOpen())
        return false;

    const int ret = wiringPiI2CRawWrite(m_Fd, cmd, 2);
    if (ret != 2)
    {
        const int err = errno;

        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "SHT raw write failed: fd=%d errno=%d (%s)",
                     m_Fd, err, std::strerror(err));

        close();
        return false;
    }

    return true;
}

bool SHTReader::readMeasurement(Readings &out)
{
    if (!ensureOpen())
        return false;

    uint8_t buf[6] = {0};

    const int ret = wiringPiI2CRawRead(m_Fd, buf, 6);
    if (ret != 6)
    {
        const int err = errno;

        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "SHT raw read failed: fd=%d errno=%d (%s)",
                     m_Fd, err, std::strerror(err));

        close();
        return false;
    }

    const uint8_t tempCrc = crc8(buf, 2);
    const uint8_t humCrc = crc8(buf + 3, 2);

    if (tempCrc != buf[2] || humCrc != buf[5])
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "SHT CRC error: temp_crc=0x%02X expected=0x%02X, hum_crc=0x%02X expected=0x%02X",
                     buf[2], tempCrc, buf[5], humCrc);
        return false;
    }

    const uint16_t rawTemp =
        (static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]);

    const uint16_t rawHumidity =
        (static_cast<uint16_t>(buf[3]) << 8) | static_cast<uint16_t>(buf[4]);

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
    return true;
}