#include "powermonitor.h"
#include "boardio.h"

#include <algorithm>

PowerMonitor::PowerMonitor(BoardIO &boardIO)
    : m_BoardIO(boardIO)
{
}

PowerMonitor::~PowerMonitor() = default;

void PowerMonitor::setCalibration(const Calibration &calibration)
{
    m_Calibration = calibration;
}

PowerMonitor::Calibration PowerMonitor::calibration() const
{
    return m_Calibration;
}

PowerMonitor::PowerReadout PowerMonitor::readPower(int voltageChannel, int currentChannel) const
{
    PowerReadout result;

    const int rawVoltage = readAdc(voltageChannel);
    const int rawCurrent = readAdc(currentChannel);

    if (rawVoltage < 0 || rawCurrent < 0)
        return result;

    result.rawVoltage = rawVoltage;
    result.rawCurrent = rawCurrent;

    result.sensedVoltage = adcToVoltage(rawVoltage);
    result.inputVoltage = calculateInputVoltage(result.sensedVoltage);

    const double currentSenseVoltage = adcToVoltage(rawCurrent);
    result.current = calculateCurrent(currentSenseVoltage);

    if (result.current < 0.0)
        result.current = 0.0;

    result.power = result.inputVoltage * result.current;
    result.valid = true;

    return result;
}

int PowerMonitor::readAdc(int channel) const
{
    // To jest celowo tylko wzorzec.
    // Tu podmienisz logikę na swój realny odczyt ADC:
    // - I2C ADC
    // - SPI ADC
    // - inny układ pomiarowy

    if (!m_BoardIO.isConnected())
        return -1;

    // Przykład-szkielet:
    // return m_BoardIO.readAdc(channel);

    (void)channel;
    return -1;
}

double PowerMonitor::adcToVoltage(int rawAdc) const
{
    if (m_Calibration.adcMaxValue <= 0)
        return 0.0;

    return (static_cast<double>(rawAdc) / static_cast<double>(m_Calibration.adcMaxValue))
           * m_Calibration.adcReferenceVoltage;
}

double PowerMonitor::calculateInputVoltage(double sensedVoltage) const
{
    return sensedVoltage * m_Calibration.voltageDividerRatio;
}

double PowerMonitor::calculateCurrent(double sensedVoltage) const
{
    const double correctedVoltage = sensedVoltage - m_Calibration.currentOffsetVoltage;
    return correctedVoltage * m_Calibration.currentScaleAperV;
}