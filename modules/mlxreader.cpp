#include "mlxreader.h"

#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <indilogger.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

namespace
{
    static constexpr uint8_t MLX90614_REG_TA = 0x06;
    static constexpr uint8_t MLX90614_REG_TOBJ1 = 0x07;

    static double mlxRawToCelsius(uint16_t raw)
    {
        return static_cast<double>(raw) * 0.02 - 273.15;
    }
}

MLXReader::MLXReader(const std::string &deviceName)
    : BaseComponent(deviceName, "MLXReader")
{
}

MLXReader::~MLXReader()
{
    close();
}

bool MLXReader::open()
{
    close();

    m_Fd = wiringPiI2CSetup(m_MlxAddress);
    if (m_Fd < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "MLX I2C setup failed: addr=0x%02X errno=%d (%s)",
                     m_MlxAddress, errno, std::strerror(errno));
        return false;
    }

    return true;
}

bool MLXReader::ensureOpen()
{
    if (isOpen())
        return true;
    return open();
}

void MLXReader::close()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;
    }
}

bool MLXReader::isOpen() const
{
    return m_Fd >= 0;
}

bool MLXReader::readWord(uint8_t reg, uint16_t &value)
{
    if (!ensureOpen())
        return false;

    const int ret = wiringPiI2CReadReg16(m_Fd, reg);
    if (ret < 0)
    {
        const int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "MLX read word reg 0x%02X failed: fd=%d errno=%d (%s)",
                     reg, m_Fd, err, std::strerror(err));
        close();
        return false;
    }

    value = static_cast<uint16_t>(ret);

    return true;
}

bool MLXReader::read(MLXReader::Readings &out)
{
    out = m_LastReadings;

    uint16_t rawAmbient = 0;
    uint16_t rawObject = 0;

    if (!readWord(MLX90614_REG_TA, rawAmbient))
        return false;

    if (!readWord(MLX90614_REG_TOBJ1, rawObject))
        return false;

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                 "MLX raw ambient=0x%04X object=0x%04X",
                 rawAmbient, rawObject);

    const double ambient = mlxRawToCelsius(rawAmbient);
    const double object = mlxRawToCelsius(rawObject);

    if (ambient < -70.0 || ambient > 150.0 ||
        object < -70.0 || object > 500.0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "MLX invalid temperature values: ambient=%.2fC object=%.2fC",
                     ambient, object);
        return false;
    }

    out.ambientTemperature = ambient;
    out.objectTemperature = object;
    out.tempDifference = ambient - object;

    m_LastReadings = out;

    return true;
}