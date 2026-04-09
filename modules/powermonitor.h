#pragma once

#include "i2cbus.h"
#include <cstdint>
#include <string>

class PowerMonitor
{
public:
    enum class Status
    {
        Ok,
        Busy,
        Alert
    };

    struct Readings
    {
        float vin = 0.0f;
        float vreg = 0.0f;
        float itot = 0.0f;
        float ptot = 0.0f;
        float ah = 0.0f;
        float wh = 0.0f;
    };

    // acsType:
    // 0 -> współczynnik 20
    // 1 -> współczynnik 10.8
    explicit PowerMonitor(uint8_t deviceAddress = 0x48, int acsType = 0);

    bool open(int busNumber = 1);
    bool isOpen() const;
    void close();

    // Jedno wywołanie wykonuje jeden krok sekwencji:
    // 0 trigger Vin
    // 1 read Vin
    // 2 trigger Vreg
    // 3 read Vreg
    // 4 trigger Itot
    // 5 read Itot
    bool update();

    const Readings& getReadings() const;
    Status getStatus() const;
    std::string getLastError() const;

    void resetEnergy();
    int getPowerIndex() const;

private:
    I2CBus bus_;
    int powerIndex_;
    int acsType_;

    float energyAs_;
    float energyWs_;

    Readings readings_;
    Status status_;
    std::string lastError_;

    bool triggerConversion(uint8_t configMsb);
    bool readConversionRegister(int16_t& value);
    void setError(const std::string& error);
};

	// 	/*
	// 	powerIndex 0-1 Vin WR, 2-3 Vreg WR, 4-5 Itot WR

	// 	15 		- 1 	start single conv
	// 	14:12	- 100 	Vin, 101 Vreg, 110 Itot, 111 Iref, 011 Ireal
	// 	11:9  	- 001	+-4.096V
	// 	8		- 1 single

	// 	7:5		- 010 32SPS, 011 64SPS, 001 16SPS
	// 	4:2		- 000 comparator
	// 	1:0		- 11 comparator disable
	// 	*/