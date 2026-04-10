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
    static constexpr uint8_t REG_CONVERSION = 0x00;
    static constexpr uint8_t REG_CONFIG     = 0x01;

    // ADS1115 PGA ±4.096V, single-shot, 128 SPS
    static constexpr uint16_t ADS_BASE_CONFIG = 0x0183;

    enum class Channel : uint8_t
    {
        VIN  = 0,
        VREG = 1,
        ACS  = 2
    };

    bool readChannel(Channel channel, double &voltage);
    bool writeRegister16(uint8_t reg, uint16_t value);
    bool readRegister16(uint8_t reg, uint16_t &value);
    static uint16_t swap16(uint16_t value);

    double adcToVoltage(int16_t raw) const;
    double adcToCurrent(double sensorVoltage) const;
    std::string busPath(int bus) const;

private:
    int m_Fd = -1;
    uint8_t m_AdsAddress = 0x48;
    uint8_t m_AcsType = 0; // 0 = ACS 20A, 1 = ACS 5A

    bool m_HaveEnergyBaseline = false;
    uint64_t m_LastSampleMs = 0;
    double m_Ah = 0.0;
    double m_Wh = 0.0;
};