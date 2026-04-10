#include "powermonitor.h"

#include <wiringPi.h>
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
    close();
    //m_Fd = wiringPiI2CSetup(bus);
    m_Fd = 1;
    return m_Fd >= 0;
}

void PowerMonitor::close()
{
    if (m_Fd >= 0)
    {
        // ::close(m_Fd);
        m_Fd = -1;
    }
}

bool PowerMonitor::isOpen() const
{
    return m_Fd >= 0;
}

bool PowerMonitor::read(PowerMonitor::Readings &out)
{
    if (wiringPiSetup() == -1)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Error %d", 1);
        return 0;
    }

    if (ads1115Setup(100, 0x48) == 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Error %d", 2);
        return 0;
    }

    int raw0 = analogRead(100 + 0);
    int raw1 = analogRead(100 + 1);
    int raw2 = analogRead(100 + 2);
    int raw3 = analogRead(100 + 3);

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Readings %d %d %d %d", raw0, raw1, raw2, raw3);

    // if (!isOpen())
    //     return false;

    // uint8_t writeBuf[3];
    // uint8_t readBuf[2];

    /*
    powerIndex 0-1 Vin WR, 2-3 Vreg WR, 4-5 Itot WR

    15 		- 1 	start single conv
    14:12	- 100 	Vin, 101 Vreg, 110 Itot, 111 Iref, 011 Ireal
    11:9  	- 001	+-4.096V
    8		- 1 single

    7:5		- 010 32SPS, 011 64SPS, 001 16SPS
    4:2		- 000 comparator
    1:0		- 11 comparator disable
    */

    // writeBuf[0] = 0x01;
    // writeBuf[1] = 0b11000011;
    // writeBuf[2] = 0b00100011;

    // if ((powerIndex % 2) == 0) // Trigger conversion
    // {
    //     switch (powerIndex)
    //     {
    //     case 0:
    //         writeBuf[1] = 0b11000011;
    //         break;
    //     case 2:
    //         writeBuf[1] = 0b11010011;
    //         break;
    //     case 4:
    //         writeBuf[1] = 0b10110011;
    //         break;
    //     }
    //     uint16_t config = (writeBuf[1] << 8) | writeBuf[2];
    //     int written = wiringPiI2CWriteReg16(m_Fd, 0x01, __bswap_16(config));
    //     DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "written 1 %d", written);

    // }
    // else // Trigger read
    // {
    //     writeBuf[0] = 0x00;
    //     int written = wiringPiI2CRawWrite(m_Fd, writeBuf, 1);
    //     DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "written 2 %d", written);
    //     if (written == 0)
    //     {
    //         int read = wiringPiI2CRawRead(m_Fd, readBuf, 2);
    //         DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "read %d", read);
    //         if (read > 0)
    //         {
    //             // int16_t val = readBuf[0] * 255 + readBuf[1];
    //             int16_t val = (static_cast<int16_t>(static_cast<unsigned char>(readBuf[0])) << 8) | static_cast<unsigned char>(readBuf[1]);

    //             switch (powerIndex)
    //             {
    //             case 1:
    //                 out.vin = (float)val / 32768.0 * 4.096 * 6.6;
    //                 break;
    //             case 3:
    //                 out.vreg = (float)val / 32768.0 * 4.096 * 6.6;
    //                 break;
    //             case 5:
    //                 out.current = (float)val / 32768.0 * 4.096 * 1 * ((m_AcsType == 0) ? 20 : 10.8);
    //                 break;
    //             }
    //             out.power = out.vin * out.current;
    //             energyAs += out.current * 0.4;
    //             energyWs += out.vin * out.current * 0.4;
    //             out.ah = energyAs / 3600;
    //             out.wh = energyWs / 3600;
    //         }
    //     }
    // }
    // powerIndex++;
    // if (powerIndex > 5)
    //     powerIndex = 0;

    return true;
}
