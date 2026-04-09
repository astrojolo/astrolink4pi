#include "i2cbus.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <indilogger.h>

I2CBus::I2CBus(std::string devicePath, uint8_t address)
    : m_DevicePath(std::move(devicePath)), m_Address(address)
{
}

I2CBus::~I2CBus()
{
    closeBus();
}

bool I2CBus::openBus()
{
    if (m_Fd >= 0)
        return true;

    m_Fd = ::open(m_DevicePath.c_str(), O_RDWR);
    if (m_Fd < 0)
    {
        DEBUGFDEVICE("AstroLink 4 Pi I2C", INDI::Logger::DBG_ERROR, "I2C: cannot open %s: %s",
               m_DevicePath.c_str(), std::strerror(errno));
        return false;
    }

    if (!selectSlave())
    {
        closeBus();
        return false;
    }

    return true;
}

void I2CBus::closeBus()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;
    }
}

bool I2CBus::isOpen() const
{
    return m_Fd >= 0;
}

bool I2CBus::selectSlave()
{
    if (m_Fd < 0)
        return false;

    if (::ioctl(m_Fd, I2C_SLAVE, m_Address) < 0)
    {
        DEBUGFDEVICE("AstroLink 4 Pi I2C", INDI::Logger::DBG_ERROR, "I2C: ioctl(I2C_SLAVE, 0x%02X) failed: %s",
               m_Address, std::strerror(errno));
        return false;
    }

    return true;
}

bool I2CBus::writeBytes(const uint8_t *data, size_t len)
{
    if (!openBus())
        return false;

    const ssize_t written = ::write(m_Fd, data, len);
    if (written != static_cast<ssize_t>(len))
    {
        DEBUGFDEVICE("AstroLink 4 Pi I2C", INDI::Logger::DBG_ERROR, "I2C: write failed: %s", std::strerror(errno));
        return false;
    }

    return true;
}

bool I2CBus::readBytes(uint8_t *data, size_t len)
{
    if (!openBus())
        return false;

    const ssize_t rd = ::read(m_Fd, data, len);
    if (rd != static_cast<ssize_t>(len))
    {
        DEBUGFDEVICE("AstroLink 4 Pi I2C", INDI::Logger::DBG_ERROR, "I2C: read failed: %s", std::strerror(errno));
        return false;
    }

    return true;
}

bool I2CBus::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return writeBytes(buf, sizeof(buf));
}

bool I2CBus::readRegister(uint8_t reg, uint8_t &value)
{
    if (!writeBytes(&reg, 1))
        return false;

    return readBytes(&value, 1);
}

bool I2CBus::readRegister16(uint8_t reg, uint16_t &value, bool swapBytes)
{
    uint8_t data[2] = {0, 0};

    if (!writeBytes(&reg, 1))
        return false;

    if (!readBytes(data, 2))
        return false;

    uint16_t raw = static_cast<uint16_t>(data[0] << 8) | data[1];

    if (swapBytes)
        raw = static_cast<uint16_t>((raw >> 8) | (raw << 8));

    value = raw;
    return true;
}