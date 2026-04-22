#include "tslreader.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <unistd.h>

#include <indilogger.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

TSLReader::TSLReader(const std::string &deviceName)
    : BaseComponent(deviceName, "TSLReader")
{
}

TSLReader::~TSLReader()
{
    close();
}

bool TSLReader::open()
{
    close();

    m_Fd = wiringPiI2CSetup(m_TslAddress);
    if (m_Fd < 0)
    {
        const int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "TSL I2C setup failed: addr=0x%02X errno=%d (%s)",
                     m_TslAddress, err, std::strerror(err));
        return false;
    }

    resetAcquisitionState();
    m_Mode = TSLState::NotAvailable;

    return true;
}

void TSLReader::close()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;
    }

    resetAcquisitionState();
    m_Mode = TSLState::NotAvailable;
}

void TSLReader::resetAcquisitionState()
{
    m_AdcStartTime = 0;
    m_NIter = 0;
    m_FullCumulative = 0;
    m_IrCumulative = 0;
}

bool TSLReader::isOpen() const
{
    return m_Fd >= 0;
}

bool TSLReader::ensureOpen()
{
    if (isOpen())
        return true;

    return open();
}

bool TSLReader::readReg8(uint8_t reg, uint8_t &value)
{
    if (!ensureOpen())
        return false;

    const int ret = wiringPiI2CReadReg8(m_Fd, reg);
    if (ret < 0)
    {
        const int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "TSL read reg8 0x%02X failed: fd=%d errno=%d (%s)",
                     reg, m_Fd, err, std::strerror(err));
        close();
        return false;
    }

    value = static_cast<uint8_t>(ret);
    return true;
}

bool TSLReader::writeReg8(uint8_t reg, uint8_t value)
{
    if (!ensureOpen())
        return false;

    if (wiringPiI2CWriteReg8(m_Fd, reg, value) < 0)
    {
        const int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "TSL write reg8 0x%02X failed: fd=%d errno=%d (%s)",
                     reg, m_Fd, err, std::strerror(err));
        close();
        return false;
    }

    return true;
}

bool TSLReader::readReg16(uint8_t reg, uint16_t &value)
{
    if (!ensureOpen())
        return false;

    const int ret = wiringPiI2CReadReg16(m_Fd, reg);
    if (ret < 0)
    {
        const int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "TSL read reg16 0x%02X failed: fd=%d errno=%d (%s)",
                     reg, m_Fd, err, std::strerror(err));
        close();
        return false;
    }

    value = static_cast<uint16_t>(ret);
    return true;
}

bool TSLReader::probeSensor()
{
    int rc = wiringPiI2CWrite(m_Fd, 0x80 | 0x20 | 0x12);
    if (rc < 0)
    {
        int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "TSL probe failed: fd=%d errno=%d (%s)",
                     m_Fd, err, std::strerror(err));
        return false;
    }

    return true;
}

bool TSLReader::initializeSensor()
{
    if (!writeReg8(
            TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
            TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN))
        return false;

    if (!writeReg8(
            TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL,
            0x05 | 0x30))
        return false;

    if (!writeReg8(
            TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
            TSL2591_ENABLE_POWEROFF))
        return false;

    return true;
}

bool TSLReader::startIntegration()
{
    return writeReg8(
        TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
        TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN);
}

bool TSLReader::stopIntegration()
{
    return writeReg8(
        TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
        TSL2591_ENABLE_POWEROFF);
}

bool TSLReader::readChannels(uint16_t &full, uint16_t &ir)
{
    uint16_t irRaw = 0;
    uint16_t fullRaw = 0;

    if (!readReg16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW, irRaw))
        return false;

    if (!readReg16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW, fullRaw))
        return false;

    ir = irRaw;
    full = fullRaw;
    return true;
}

bool TSLReader::read(TSLReader::Readings &out)
{
    out = m_LastReadings;

    if (!ensureOpen())
    {
        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                    "TSL I2C not available.");
        return false;
    }

    if (m_Mode == TSLState::NotAvailable)
    {
        if (!probeSensor())
        {
            close();
            return false;
        }

        m_Mode = TSLState::Available;
        out = m_LastReadings;
        return true;
    }

    if (m_Mode == TSLState::Available)
    {
        if (!initializeSensor())
        {
            close();
            return false;
        }

        m_Mode = TSLState::Initialized;
        out = m_LastReadings;
        return true;
    }

    if (m_Mode != TSLState::Initialized)
        return false;

    if (m_AdcStartTime == 0)
    {
        if (!startIntegration())
        {
            close();
            return false;
        }

        m_AdcStartTime = millis();
        out = m_LastReadings;
        return true;
    }

    if (static_cast<uint32_t>(millis() - m_AdcStartTime) < TSL2591_ADC_TIME)
    {
        out = m_LastReadings;
        return true;
    }

    uint16_t full = 0;
    uint16_t ir = 0;

    const bool readOk = readChannels(full, ir);
    const bool stopOk = stopIntegration();

    m_AdcStartTime = 0;

    if (!readOk || !stopOk)
    {
        close();
        return false;
    }

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                 "TSL sample: full=%u ir=%u", full, ir);

    if (full < ir)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "TSL invalid sample: full < ir (%u < %u)", full, ir);
        out = m_LastReadings;
        return true;
    }

    const uint32_t nextFull = m_FullCumulative + static_cast<uint32_t>(full);
    const uint32_t nextIr = m_IrCumulative + static_cast<uint32_t>(ir);
    const uint32_t nextVisible = nextFull - nextIr;

    if (m_NIter < 5 || (nextVisible < 500 && m_NIter < 150))
    {
        ++m_NIter;
        m_FullCumulative = nextFull;
        m_IrCumulative = nextIr;
        out = m_LastReadings;
        return true;
    }

    const uint32_t visible = m_FullCumulative - m_IrCumulative;

    if (visible > 0 && m_NIter > 0)
    {
        const double visNorm = static_cast<double>(visible) / (29628.0 * static_cast<double>(m_NIter));

        if (visNorm > 0.0)
        {
            const double mpsas =
                12.6 - 1.086 * std::log(visNorm) + m_SQMOffset + m_FilterCoeff;

            m_LastReadings.mpsas = mpsas;
            m_LastReadings.full = static_cast<int>(m_FullCumulative / m_NIter);
            m_LastReadings.ir = static_cast<int>(m_IrCumulative / m_NIter);
            m_LastReadings.visible = static_cast<int>(visible / m_NIter);
            m_LastReadings.valid = true;
        }
    }

    resetAcquisitionState();
    out = m_LastReadings;
    return true;
}