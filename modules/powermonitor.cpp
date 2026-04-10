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

bool PowerMonitor::open(int bus)
{
    // close();

    int wipi = wiringPiSetup();
    if (wipi < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "WiPi open failed: errno=%d (%s)", errno, std::strerror(errno));
        return 0;
    }

    m_Fd = wiringPiI2CSetup(m_AdsAddress);
    if (m_Fd < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "I2C setup failed: errno=%d (%s)", errno, std::strerror(errno));
        return 0;
    }

    return m_Fd >= 0;
}

void PowerMonitor::close()
{
    if (m_Fd >= 0)
    {
        // m_Fd = -1;
    }
}

bool PowerMonitor::isOpen() const
{
    return m_Fd >= 0;
}

bool PowerMonitor::read(PowerMonitor::Readings &out)
{
    if (!isOpen() || m_Fd < 0)
        return false;

    // powerIndex:
    // 0 - start Vin
    // 1 - read  Vin
    // 2 - start Vreg
    // 3 - read  Vreg
    // 4 - start Itot
    // 5 - read  Itot

    uint16_t config = 0;

    switch (powerIndex)
    {
    case 0: // start Vin, MUX=100
        config = 0xC323;
        break;

    case 2: // start Vreg, MUX=101
        config = 0xD323;
        break;

    case 4: // start Itot, MUX=011
        config = 0xB323;
        break;

    case 1:
    case 3:
    case 5:
    {
        int16_t raw = wiringPiI2CReadReg16(m_Fd, 0x00);
        if (raw < 0)
        {
            int err = errno;
            DEBUGFDEVICE(getDeviceName().c_str(),
                         INDI::Logger::DBG_SESSION,
                         "ADS1115 read failed: fd=%d errno=%d (%s)",
                         m_Fd, err, std::strerror(err));
            return false;
        }

        raw = static_cast<int16_t>(__bswap_16(static_cast<uint16_t>(raw)));

        // dla wejść single-ended wynik ujemny traktujemy jako 0
        if (raw < 0)
            raw = 0;

        const float fsr = 4.096f;
        const float adcVoltage = static_cast<float>(raw) / 32768.0f * fsr;

        switch (powerIndex)
        {
        case 1:
            out.vin = adcVoltage * 6.6f;
            break;

        case 3:
            out.vreg = adcVoltage * 6.6f;
            break;

        case 5:
            out.current = adcVoltage * ((m_AcsType == 0) ? 20.0f : 10.8f);
            out.power = out.vin * out.current;
            energyAs += out.current * 0.4f;
            energyWs += out.power * 0.4f;
            out.ah = energyAs / 3600.0f;
            out.wh = energyWs / 3600.0f;
            break;
        }

        break;
    }

    default:
        powerIndex = 0;
        return true;
    }

    // Start single conversion on even steps
    if ((powerIndex % 2) == 0)
    {
        int written = wiringPiI2CWriteReg16(m_Fd, 0x01, __bswap_16(config));
        if (written < 0)
        {
            int err = errno;
            DEBUGFDEVICE(getDeviceName().c_str(),
                         INDI::Logger::DBG_SESSION,
                         "ADS1115 write failed: fd=%d config=0x%04X errno=%d (%s)",
                         m_Fd, config, err, std::strerror(err));
            return false;
        }
    }

    powerIndex++;
    if (powerIndex > 5)
        powerIndex = 0;

    return true;
}

// bool PowerMonitor::read(PowerMonitor::Readings &out)
// {
//     if (!isOpen())
//         return false;

//     uint8_t writeBuf[3];

//     /*
//     powerIndex 0-1 Vin WR, 2-3 Vreg WR, 4-5 Itot WR

//     15 		- 1 	start single conv
//     14:12	- 100 	Vin, 101 Vreg, 110 Itot, 111 Iref, 011 Ireal
//     11:9  	- 001	+-4.096V
//     8		- 1 single

//     7:5		- 010 32SPS, 011 64SPS, 001 16SPS
//     4:2		- 000 comparator
//     1:0		- 11 comparator disable
//     */

//     writeBuf[0] = 0x01;
//     writeBuf[1] = 0b11000011;
//     writeBuf[2] = 0b00100011;

//     if ((powerIndex % 2) == 0) // Trigger conversion
//     {
//         switch (powerIndex)
//         {
//         case 0:
//             writeBuf[1] = 0b11000011;
//             break;
//         case 2:
//             writeBuf[1] = 0b11010011;
//             break;
//         case 4:
//             writeBuf[1] = 0b10110011;
//             break;
//         }
//         uint16_t config = (writeBuf[1] << 8) | writeBuf[2];
//         int written = wiringPiI2CWriteReg16(m_Fd, 0x01, __bswap_16(config));
//         if (written < 0)
//         {
//             DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "I2C write failed: errno=%d (%s)", errno, std::strerror(errno));
//         }
//     }
//     else // Trigger read
//     {
//         int16_t read = wiringPiI2CReadReg16(m_Fd, 0x00);
//         if (read < 0)
//         {
//             DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "I2C read failed: errno=%d (%s)", errno, std::strerror(errno));
//         }
//         else
//         {
//             switch (powerIndex)
//             {
//             case 1:
//                 out.vin = (float)read / 32768.0 * 4.096 * 6.6;
//                 break;
//             case 3:
//                 out.vreg = (float)read / 32768.0 * 4.096 * 6.6;
//                 break;
//             case 5:
//                 out.current = (float)read / 32768.0 * 4.096 * 1 * ((m_AcsType == 0) ? 20 : 10.8);
//                 break;
//             }
//             out.power = out.vin * out.current;
//             energyAs += out.current * 0.4;
//             energyWs += out.vin * out.current * 0.4;
//             out.ah = energyAs / 3600;
//             out.wh = energyWs / 3600;
//         }
//         // }
//     }
//     powerIndex++;
//     if (powerIndex > 5)
//         powerIndex = 0;

//     return true;
// }
