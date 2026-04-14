#include "tslreader.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <unistd.h>

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

    int wipi = wiringPiSetup();
    if (wipi < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "WiPi open failed: errno=%d (%s)",
                     errno, std::strerror(errno));
        return false;
    }

    m_Fd = wiringPiI2CSetup(m_TslAddress);
    if (m_Fd < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "TSL I2C setup failed: errno=%d (%s)",
                     errno, std::strerror(errno));
        return false;
    }

    m_Mode = TSLState::NotAvailable;
    m_AdcStartTime = 0;
    m_NIter = 0;
    m_FullCumulative = 0;
    m_IrCumulative = 0;
    m_LastReadings = Readings{};

    return true;
}

void TSLReader::close()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;
    }

    m_Mode = TSLState::NotAvailable;
    m_AdcStartTime = 0;
    m_NIter = 0;
    m_FullCumulative = 0;
    m_IrCumulative = 0;
}

bool TSLReader::isOpen() const
{
    return m_Fd >= 0;
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
    int rc = 0;

    rc |= (wiringPiI2CWriteReg8(
               m_Fd,
               TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
               TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN) < 0);

    // Enable device - power down mode on boot
    rc |= (wiringPiI2CWriteReg8(
               m_Fd,
               TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL,
               0x05 | 0x30) < 0);

    rc |= (wiringPiI2CWriteReg8(
               m_Fd,
               TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
               TSL2591_ENABLE_POWEROFF) < 0);

    if (rc != 0)
    {
        int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "TSL init failed: fd=%d errno=%d (%s)",
                     m_Fd, err, std::strerror(err));
        return false;
    }

    return true;
}

bool TSLReader::startIntegration()
{
    int rc = wiringPiI2CWriteReg8(
        m_Fd,
        TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
        TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN);

    if (rc < 0)
    {
        int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "TSL start integration failed: fd=%d errno=%d (%s)",
                     m_Fd, err, std::strerror(err));
        return false;
    }

    return true;
}

bool TSLReader::stopIntegration()
{
    int rc = wiringPiI2CWriteReg8(
        m_Fd,
        TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
        TSL2591_ENABLE_POWEROFF);

    if (rc < 0)
    {
        int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "TSL stop integration failed: fd=%d errno=%d (%s)",
                     m_Fd, err, std::strerror(err));
        return false;
    }

    return true;
}

bool TSLReader::readChannels(int &full, int &ir)
{
    int irRaw = wiringPiI2CReadReg16(m_Fd, TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
    if (irRaw < 0)
    {
        int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "TSL read CH1 failed: fd=%d errno=%d (%s)",
                     m_Fd, err, std::strerror(err));
        return false;
    }

    int fullRaw = wiringPiI2CReadReg16(m_Fd, TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);
    if (fullRaw < 0)
    {
        int err = errno;
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "TSL read CH0 failed: fd=%d errno=%d (%s)",
                     m_Fd, err, std::strerror(err));
        return false;
    }

    ir = irRaw;
    full = fullRaw;
    return true;
}

bool TSLReader::read(TSLReader::Readings &out)
{
    if (!isOpen())
        return false;

    out = m_LastReadings;
    bool available = false;

    if (m_Mode == TSLState::NotAvailable)
    {
        if (probeSensor())
        {
            m_Mode = TSLState::Available;
            available = true;
        }
        else
        {
            m_Mode = TSLState::NotAvailable;
            return false;
        }
    }
    else if (m_Mode == TSLState::Available)
    {
        if (initializeSensor())
        {
            m_Mode = TSLState::Initialized;
            available = true;
        }
        else
        {
            m_Mode = TSLState::NotAvailable;
            return false;
        }
    }
    else if (m_Mode == TSLState::Initialized)
    {
        if (m_AdcStartTime == 0)
        {
            if (startIntegration())
            {
                m_AdcStartTime = millis();
                available = true;
            }
            else
            {
                m_Mode = TSLState::NotAvailable;
                return false;
            }
        }
        else if (millis() > (m_AdcStartTime + TSL2591_ADC_TIME))
        {
            int ir = 0;
            int full = 0;

            bool readOk = readChannels(full, ir);
            bool stopOk = stopIntegration();

            m_AdcStartTime = 0;

            if (!readOk || !stopOk)
            {
                m_Mode = TSLState::NotAvailable;
                return false;
            }

            const int visCumulative = m_FullCumulative - m_IrCumulative;


            if (full < ir)
            {
                out = m_LastReadings;
                return true;
            }

            if (m_NIter < 5 || (visCumulative < 500 && m_NIter < 150))
            {
                ++m_NIter;
                m_FullCumulative += full;
                m_IrCumulative += ir;
            }
            else
            {
                const int visible = m_FullCumulative - m_IrCumulative;

                if (visible > 0 && m_NIter > 0)
                {
                    const double VIS = static_cast<double>(visible) / (29628.0 * m_NIter);
                    const double mpsas =
                        12.6 - 1.086 * std::log(VIS) + m_SQMOffset + m_FilterCoeff;

                    m_LastReadings.mpsas = mpsas;
                    m_LastReadings.full = full;
                    m_LastReadings.ir = ir;
                    m_LastReadings.visible = visible;
                    m_LastReadings.valid = true;

                    out = m_LastReadings;
                }

                m_NIter = 0;
                m_IrCumulative = 0;
                m_FullCumulative = 0;
            }

            available = true;
        }
        else
        {
            available = true;
        }
    }

    out = m_LastReadings;
    return available;
}