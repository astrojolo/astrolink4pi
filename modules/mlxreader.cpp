#include "mlxreader.h"

#include <cerrno>
#include <cstring>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

namespace
{
static constexpr uint8_t MLX90614_REG_TA    = 0x06; // Ambient temperature
static constexpr uint8_t MLX90614_REG_TOBJ1 = 0x07; // Object temperature

static double mlxRawToCelsius(uint16_t raw)
{
    // MLX90614: 0.02 K/LSB, offset -273.15 C
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

    const int wipi = wiringPiSetup();
    if (wipi < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "WiPi open failed: errno=%d (%s)",
                     errno, std::strerror(errno));
        return false;
    }

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

    // wiringPiI2CReadReg8() pozwala złożyć słowo jawnie w poprawnej kolejności
    // bez zgadywania endianowości wiringPiI2CReadReg16().
    const int lsb = wiringPiI2CReadReg8(m_Fd, reg);
    if (lsb < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "MLX read reg 0x%02X LSB failed: fd=%d errno=%d (%s)",
                     reg, m_Fd, errno, std::strerror(errno));
        return false;
    }

    const int msb = wiringPiI2CReadReg8(m_Fd, reg + 1);
    if (msb < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "MLX read reg 0x%02X MSB failed: fd=%d errno=%d (%s)",
                     reg, m_Fd, errno, std::strerror(errno));
        return false;
    }

    value = static_cast<uint16_t>(
        (static_cast<uint16_t>(msb & 0xFF) << 8) |
        static_cast<uint16_t>(lsb & 0xFF));

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
                 "MLX raw ambient=%u object=%u",
                 rawAmbient, rawObject);

    out.ambientTemperature = mlxRawToCelsius(rawAmbient);
    out.objectTemperature  = mlxRawToCelsius(rawObject);
    out.tempDifference = out.objectTemperature - out.ambientTemperature;

    m_LastReadings = out;

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION,
                 "MLX temp ambient=%.2fC object=%.2fC",
                 out.ambientTemperature, out.objectTemperature);

    return true;
}