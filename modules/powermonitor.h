#pragma once

#include <cstdint>
#include <string>

class PowerMonitor
{
public:
    struct Readings
    {
        double vin = 0.0;
        double vreg = 0.0;
        double current = 0.0;
        double power = 0.0;
        double ah = 0.0;
        double wh = 0.0;
    };

    PowerMonitor(uint8_t adsAddress, uint8_t acsType);
    ~PowerMonitor();

    bool open(int bus);
    void close();
    bool isOpen() const;

    bool read(Readings &out);


private:
    int m_Fd = -1;
    uint8_t m_AdsAddress = 0x48;
    uint8_t m_AcsType = 0; // 0 = ACS 20A, 1 = ACS 5A
};