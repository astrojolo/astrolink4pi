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
    close();
    // m_Fd = wiringPiI2CSetup(bus);
    if (wiringPiSetup() == -1)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Error %d", 1);
        return 0;
    }

    int m_Fd = wiringPiI2CSetup(0x48);
    if (m_Fd < 0)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Error %d", 2);
        return 0;
    }

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

    // CONFIG:
    // OS=1 start single conversion
    // MUX=100 AIN0 względem GND
    // PGA=001 ±4.096V
    // MODE=1 single-shot
    // DR=100 128SPS
    // COMP_QUE=11 disable comparator
    uint16_t config = 0xC383;

    int written = wiringPiI2CWriteReg16(m_Fd, 0x01, __bswap_16(config));

    DEBUGFDEVICE(getDeviceName().c_str(),
             INDI::Logger::DBG_SESSION,
             "I2C write failed: errno=%d (%s)",
             errno,
             std::strerror(errno));

    // czekaj aż konwersja się skończy
    while (true)
    {
        int16_t cfg = wiringPiI2CReadReg16(m_Fd, 0x01);
        cfg = __bswap_16(cfg);
        if (cfg & 0x8000)
            break;
        delayMicroseconds(100);
    }

    int16_t raw = wiringPiI2CReadReg16(m_Fd, 0x00);
    raw = __bswap_16(raw);

    if (raw < 0)
        raw = 0; // dla single-ended bywa praktyczne

    double voltage = raw * 4.096 / 32768.0;

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Readings %d %d %d %f", m_Fd, written, raw, voltage);

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
