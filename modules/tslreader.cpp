#include "tslreader.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <unistd.h>

#include "log_macros.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>

namespace
{
    static constexpr uint8_t TSL2591_REG_ID = 0x12;
}

TSLReader::TSLReader(const std::string &deviceName)
    : BaseComponent(deviceName, "TSLReader")
{
    resetAcquisitionState();
    m_Configured = false;
}

TSLReader::~TSLReader()
{
    close();
}

void TSLReader::resetAcquisitionState()
{
    m_AdcStartTime = 0;
    m_NIter = 0;
    m_FullCumulative = 0;
    m_IrCumulative = 0;
}

void TSLReader::invalidateSensorState()
{
    resetAcquisitionState();
    m_Configured = false;
    m_LastReadings.valid = false;
}

bool TSLReader::open()
{
    if (m_Fd >= 0)
        return true;

    m_Fd = wiringPiI2CSetup(m_TslAddress);
    if (m_Fd < 0)
    {
        const int err = errno;
        DEBUGFDEVICE_LOG_ONCE(m_warnLogged, getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "TSL I2C setup failed: addr=0x%02X errno=%d (%s)",
                     m_TslAddress, err, std::strerror(err));
        m_Fd = -1;
        return false;
    }

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                 "TSL I2C opened: fd=%d addr=0x%02X",
                 m_Fd, m_TslAddress);

    invalidateSensorState();
    return true;
}

void TSLReader::close()
{
    if (m_Fd >= 0)
    {
        ::close(m_Fd);
        m_Fd = -1;

        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                    "TSL I2C closed.");
    }

    invalidateSensorState();
}

bool TSLReader::isOpen() const
{
    return m_Fd >= 0;
}

bool TSLReader::ensureOpen()
{
    if (isOpen())
        return true;

    DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                "TSL attempting I2C reopen.");

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
        DEBUGFDEVICE_LOG_ONCE(m_warnLogged, getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
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
        DEBUGFDEVICE_LOG_ONCE(m_warnLogged, getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
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
        DEBUGFDEVICE_LOG_ONCE(m_warnLogged, getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
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
    uint8_t id = 0;
    if (!readReg8(TSL2591_COMMAND_BIT | TSL2591_REG_ID, id))
        return false;

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                 "TSL probe OK, sensor ID=0x%02X", id);

    return true;
}

bool TSLReader::configureSensor()
{
    if (!probeSensor())
        return false;

    if (!writeReg8(
            TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
            TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN))
    {
        return false;
    }

    if (!writeReg8(
            TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL,
            0x05 | 0x30))
    {
        return false;
    }

    if (!writeReg8(
            TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
            TSL2591_ENABLE_POWEROFF))
    {
        return false;
    }

    m_Configured = true;

    DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                "TSL sensor configured.");

    return true;
}

bool TSLReader::ensureConfigured()
{
    if (!ensureOpen())
        return false;

    if (m_Configured)
    {
        // lekki heartbeat: jeśli sensor zniknął fizycznie, wykryj to tutaj
        uint8_t id = 0;
        if (!readReg8(TSL2591_COMMAND_BIT | TSL2591_REG_ID, id))
            return false;

        return true;
    }

    return configureSensor();
}

bool TSLReader::startIntegration()
{
    if (!ensureConfigured())
        return false;

    if (!writeReg8(
            TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
            TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN))
    {
        return false;
    }

    m_AdcStartTime = millis();

    DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                "TSL integration started.");

    return true;
}

bool TSLReader::stopIntegration()
{
    if (!isOpen())
        return false;

    if (!writeReg8(
            TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
            TSL2591_ENABLE_POWEROFF))
    {
        return false;
    }

    return true;
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
    // Domyślnie zwracamy ostatnie znane dane.
    out = m_LastReadings;

    // Brak komunikacji z sensorem.
    if (!ensureConfigured())
    {
        out = TSLReader::Readings{};
        out.valid = false;
        return false;
    }

    // Komunikacja jest OK, ale integracja jeszcze nie została uruchomiona.
    if (m_AdcStartTime == 0)
    {
        if (!startIntegration())
        {
            out = TSLReader::Readings{};
            out.valid = false;
            return false;
        }

        // Integracja właśnie wystartowała.
        // Jeśli nie ma jeszcze żadnych poprawnych danych historycznych,
        // out.valid ma pozostać false.
        out = m_LastReadings;        
        return true;
    }

    const uint32_t elapsed = static_cast<uint32_t>(millis() - m_AdcStartTime);
    if (elapsed < TSL2591_ADC_TIME)
    {
        // Integracja trwa, komunikacja jest poprawna.
        // Zwracamy ostatnie znane dane (jeśli były).
        out = m_LastReadings;
        m_warnLogged = false;
        return true;
    }

    uint16_t full = 0;
    uint16_t ir = 0;

    const bool readOk = readChannels(full, ir);

    bool stopOk = false;
    if (readOk)
        stopOk = stopIntegration();

    m_AdcStartTime = 0;

    if (!readOk || !stopOk)
    {
        close();
        out = TSLReader::Readings{};
        out.valid = false;
        return false;
    }

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                 "TSL sample: full=%u ir=%u",
                 static_cast<unsigned>(full),
                 static_cast<unsigned>(ir));

    if (full < ir)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "TSL invalid sample: full < ir (%u < %u)",
                     static_cast<unsigned>(full),
                     static_cast<unsigned>(ir));

        // Komunikacja działa, ale próbka jest błędna logicznie.
        // Nie traktujemy tego jako brak komunikacji.
        // Zwracamy ostatnie poprawne dane, jeśli istnieją.
        out = m_LastReadings;
        return true;
    }

    const uint32_t nextFull = m_FullCumulative + static_cast<uint32_t>(full);
    const uint32_t nextIr = m_IrCumulative + static_cast<uint32_t>(ir);
    const uint32_t nextVisible = nextFull - nextIr;
    const uint32_t nextNIter = m_NIter + 1;

    m_FullCumulative = nextFull;
    m_IrCumulative = nextIr;
    m_NIter = nextNIter;

    // Nadal zbieramy próbki - komunikacja OK, ale nowego pełnego wyniku jeszcze nie ma.
    if (m_NIter < 5 || (nextVisible < 500 && m_NIter < 150))
    {
        out = m_LastReadings;
        return true;
    }

    const uint32_t visible = nextVisible;

    if (visible > 0)
    {
        const double visNorm =
            static_cast<double>(visible) / (29628.0 * static_cast<double>(m_NIter));

        if (visNorm > 0.0)
        {
            m_LastReadings.mpsas =
                12.6 - 1.086 * std::log(visNorm) + m_SQMOffset + m_FilterCoeff;
            m_LastReadings.full = static_cast<int>(m_FullCumulative / m_NIter);
            m_LastReadings.ir = static_cast<int>(m_IrCumulative / m_NIter);
            m_LastReadings.visible = static_cast<int>(visible / m_NIter);
            m_LastReadings.valid = true;

            out = m_LastReadings;

            DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                         "TSL reading OK: mpsas=%.3f full=%d ir=%d vis=%d n=%u",
                         m_LastReadings.mpsas,
                         m_LastReadings.full,
                         m_LastReadings.ir,
                         m_LastReadings.visible,
                         static_cast<unsigned>(m_NIter));

            resetAcquisitionState();
            return true;
        }
    }

    // Komunikacja była poprawna, ale z tej serii nie udało się wyliczyć nowego wyniku.
    // Zachowujemy poprzednie dane, jeśli są.
    out = m_LastReadings;
    m_warnLogged = false;
    resetAcquisitionState();
    return true;
}
