#include "powermonitor.h"

PowerMonitor::PowerMonitor(uint8_t deviceAddress, int acsType)
    : bus_(deviceAddress),
      powerIndex_(0),
      acsType_(acsType),
      energyAs_(0.0f),
      energyWs_(0.0f),
      status_(Status::Busy)
{
}

bool PowerMonitor::open(int busNumber)
{
    if (!bus_.open(busNumber))
    {
        setError(bus_.getLastError());
        status_ = Status::Alert;
        return false;
    }

    lastError_.clear();
    return true;
}

bool PowerMonitor::isOpen() const
{
    return bus_.isOpen();
}

void PowerMonitor::close()
{
    bus_.close();
}

bool PowerMonitor::update()
{
    if (!bus_.isOpen())
    {
        setError("I2C bus is not open");
        status_ = Status::Alert;
        return false;
    }

    bool result = false;

    if ((powerIndex_ % 2) == 0)
    {
        // Trigger conversion
        uint8_t configMsb = 0b11000011;

        switch (powerIndex_)
        {
            case 0: // Vin
                configMsb = 0b11000011;
                break;
            case 2: // Vreg
                configMsb = 0b11010011;
                break;
            case 4: // Itot
                configMsb = 0b10110011;
                break;
            default:
                configMsb = 0b11000011;
                break;
        }

        result = triggerConversion(configMsb);
        status_ = result ? Status::Busy : Status::Alert;
    }
    else
    {
        // Read result
        int16_t val = 0;
        result = readConversionRegister(val);

        if (result)
        {
            switch (powerIndex_)
            {
                case 1:
                    readings_.vin = static_cast<float>(val) / 32768.0f * 4.096f * 6.6f;
                    break;

                case 3:
                    readings_.vreg = static_cast<float>(val) / 32768.0f * 4.096f * 6.6f;
                    break;

                case 5:
                    readings_.itot = static_cast<float>(val) / 32768.0f * 4.096f *
                                     ((acsType_ == 0) ? 20.0f : 10.8f);
                    break;

                default:
                    break;
            }

            readings_.ptot = readings_.vin * readings_.itot;

            // analogicznie do Twojego kodu: krok 0.4 s
            energyAs_ += readings_.itot * 0.4f;
            energyWs_ += readings_.vin * readings_.itot * 0.4f;

            readings_.ah = energyAs_ / 3600.0f;
            readings_.wh = energyWs_ / 3600.0f;

            status_ = Status::Ok;
        }
        else
        {
            status_ = Status::Alert;
        }
    }

    powerIndex_++;
    if (powerIndex_ > 5)
        powerIndex_ = 0;

    return result;
}

const PowerMonitor::Readings& PowerMonitor::getReadings() const
{
    return readings_;
}

PowerMonitor::Status PowerMonitor::getStatus() const
{
    return status_;
}

std::string PowerMonitor::getLastError() const
{
    return lastError_;
}

void PowerMonitor::resetEnergy()
{
    energyAs_ = 0.0f;
    energyWs_ = 0.0f;
    readings_.ah = 0.0f;
    readings_.wh = 0.0f;
}

int PowerMonitor::getPowerIndex() const
{
    return powerIndex_;
}

bool PowerMonitor::triggerConversion(uint8_t configMsb)
{
    // Rejestr konfiguracyjny = 0x01
    // LSB zgodnie z Twoim kodem = 0b00100011
    char config[2];
    config[0] = static_cast<char>(configMsb);
    config[1] = static_cast<char>(0b00100011);

    const int written = bus_.writeRegister(0x01, config, 2);
    if (written != 2)
    {
        setError("Cannot write configuration to power sensor: " + bus_.getLastError());
        return false;
    }

    lastError_.clear();
    return true;
}

bool PowerMonitor::readConversionRegister(int16_t& value)
{
    char readBuf[2] = {0, 0};

    // Rejestr konwersji = 0x00
    const int read = bus_.readRegister(0x00, readBuf, 2);
    if (read != 2)
    {
        setError("Cannot read conversion register from power sensor: " + bus_.getLastError());
        return false;
    }

    value = static_cast<int16_t>(
        (static_cast<uint16_t>(static_cast<unsigned char>(readBuf[0])) << 8) |
         static_cast<uint16_t>(static_cast<unsigned char>(readBuf[1]))
    );

    lastError_.clear();
    return true;
}

void PowerMonitor::setError(const std::string& error)
{
    lastError_ = error;
}