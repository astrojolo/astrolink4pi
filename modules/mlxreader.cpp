#include "mlxreader.h"

#include <cerrno>
#include <cstring>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

namespace
{
static constexpr uint8_t MLX90614_REG_TA    = 0x06;
static constexpr uint8_t MLX90614_REG_TOBJ1 = 0x07;

static double mlxRawToCelsius(uint16_t raw)
{
    return static_cast<double>(raw) * 0.02 - 273.15;
}
}

MLXReader::MLXReader(uint8_t mlxAddress, const std::string &deviceName)
    : BaseComponent(deviceName, "MLXReader"), m_MlxAddress(mlxAddress)
{
}

MLXReader::~MLXReader()
{
    close();
}

bool MLXReader::open()
{
    close();

    static bool wiringPiInitialized = false;
    if (!wiringPiInitialized)
    {
        if (wiringPiSetup() < 0)
        {
            DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                         "wiringPiSetup failed: errno=%d (%s)",
                         errno, std::strerror(errno));
            return false;
        }
        wiringPiInitialized = true;
    }

    m_Fd = wiringPiI2CSetup(m_MlxAddress);
    if (m_Fd < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "MLX I2C setup failed: addr=0x%02X errno=%d (%s)",
                     m_MlxAddress, errno, std::strerror(errno));
        return false;
    }

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                 "MLX I2C setup success: addr=0x%02X fd=%d",
                 m_MlxAddress, m_Fd);

    return true;
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
    if (!isOpen())
        return false;

    errno = 0;
    const int ret = wiringPiI2CReadReg16(m_Fd, reg);
    if (ret < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "MLX read word reg 0x%02X failed: fd=%d errno=%d (%s)",
                     reg, m_Fd, errno, std::strerror(errno));
        return false;
    }

    // SMBus Read Word zwraca low byte + high byte.
    value = static_cast<uint16_t>(ret & 0xFFFF);

    return true;
}

bool MLXReader::read(MLXReader::Readings &out)
{
    if (!isOpen())
        return false;

    out = m_LastReadings;

    uint16_t rawAmbient = 0;
    uint16_t rawObject  = 0;

    if (!readWord(MLX90614_REG_TA, rawAmbient))
        return false;

    if (!readWord(MLX90614_REG_TOBJ1, rawObject))
        return false;

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION,
                 "MLX raw ambient=0x%04X object=0x%04X",
                 rawAmbient, rawObject);

    out.ambientTemperature = mlxRawToCelsius(rawAmbient);
    out.objectTemperature  = mlxRawToCelsius(rawObject);

    m_LastReadings = out;

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION,
                 "MLX temp ambient=%.2fC object=%.2fC",
                 out.ambientTemperature, out.objectTemperature);

    return true;
}