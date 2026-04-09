#ifndef POWERMONITOR_H
#define POWERMONITOR_H

#include <cstdint>

class BoardIO;

class PowerMonitor
{
public:
    struct Calibration
    {
        // ADC reference voltage
        double adcReferenceVoltage = 3.3;

        // ADC full scale, np. 1023 dla 10-bit, 4095 dla 12-bit
        int adcMaxValue = 1023;

        // Dzielnik napięcia:
        // Vout = Vin * (R2 / (R1 + R2))
        // więc:
        // Vin = Vout * voltageDividerRatio
        double voltageDividerRatio = 1.0;

        // Czułość toru pomiaru prądu:
        // np. ile amperów przypada na 1 V na wyjściu pomiarowym
        double currentScaleAperV = 1.0;

        // Offset napięcia dla pomiaru prądu, np. 1.65V
        double currentOffsetVoltage = 0.0;
    };

    struct PowerReadout
    {
        int rawVoltage = 0;
        int rawCurrent = 0;

        double sensedVoltage = 0.0;
        double inputVoltage = 0.0;
        double current = 0.0;
        double power = 0.0;

        bool valid = false;
    };

    explicit PowerMonitor(BoardIO &boardIO);
    ~PowerMonitor();

    void setCalibration(const Calibration &calibration);
    Calibration calibration() const;

    // Przykładowy odczyt z dwóch kanałów ADC:
    // voltageChannel - kanał ADC dla napięcia
    // currentChannel - kanał ADC dla prądu
    PowerReadout readPower(int voltageChannel, int currentChannel) const;

private:
    int readAdc(int channel) const;
    double adcToVoltage(int rawAdc) const;
    double calculateInputVoltage(double sensedVoltage) const;
    double calculateCurrent(double sensedVoltage) const;

private:
    BoardIO &m_BoardIO;
    Calibration m_Calibration;
};

#endif