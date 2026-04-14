#include "focuser.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>

#include <unistd.h>
#include <wiringPi.h>

namespace
{
    constexpr int MAX_DRIVER_CURRENT_MA = 2000;
    constexpr int MIN_DRIVER_CURRENT_MA = 0;
}

Focuser::Focuser(const Config &config, BoardIO &boardIO, PwmController &pwmController, const std::string &deviceName)
    : BaseComponent(deviceName, "Focuser"), m_BoardIO(boardIO), m_PwmController(pwmController), m_Config(config)
{
    m_State.resolution = config.defaultResolution;
    m_State.stepDelayUs = config.defaultStepDelayUs;
    m_State.currentmA = config.defaultCurrentmA;
    m_State.maxPosition = config.defaultMaxPosition;
}

Focuser::~Focuser()
{
    close();
}

bool Focuser::open()
{
    close();

    m_BoardIO.initializePin(EN_PIN, OUTPUT, HIGH); // disabled at startup
    m_BoardIO.initializePin(M0_PIN, OUTPUT, LOW);
    m_BoardIO.initializePin(M1_PIN, OUTPUT, LOW);
    m_BoardIO.initializePin(M2_PIN, OUTPUT, LOW);
    m_BoardIO.initializePin(RST_PIN, OUTPUT, HIGH);
    m_BoardIO.initializePin(STP_PIN, OUTPUT, LOW);
    m_BoardIO.initializePin(DIR_PIN, OUTPUT, LOW);
    m_BoardIO.initializePin(DECAY_PIN, OUTPUT, LOW);
    m_BoardIO.initializePin(HOLD_PIN, OUTPUT, LOW);

    if (!setResolution(m_State.resolution))
    {
        close();
        return false;
    }

    setCurrent(true);

    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        m_State.connected = true;
        m_State.moving = false;
    }

    return true;
}

void Focuser::close()
{
    abortFocuser();

    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.connected = false;
    m_State.moving = false;
}

bool Focuser::isOpen() const
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    return m_State.connected;
}

bool Focuser::abortFocuser()
{
    m_Abort.store(true, std::memory_order_relaxed);

    if (m_MotionThread.joinable())
        m_MotionThread.join();

    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        m_State.moving = false;
        m_State.targetPosition = m_State.currentPosition;
    }

    setCurrent(true);
    DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Focuser motion aborted.");
    return true;
}

bool Focuser::moveRelFocuser(int32_t ticks)
{
    State state = getState();
    return moveAbsFocuser(static_cast<uint32_t>(state.currentPosition + ticks));
}

bool Focuser::moveAbsFocuser(uint32_t targetTicks)
{
    if (!isOpen())
        return false;

    int direction = 0;
    int backlashTicksRemaining = 0;
    int32_t currentPosition = 0;
    int32_t maxPosition = 0;
    int32_t lastDirection = 0;

    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        currentPosition = m_State.currentPosition;
        maxPosition = m_State.maxPosition;
        lastDirection = m_State.lastDirection;
    }

    if (static_cast<int32_t>(targetTicks) < 0 || static_cast<int32_t>(targetTicks) > maxPosition)
    {
        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                    "Requested focuser position out of range.");
        return false;
    }

    if (static_cast<int32_t>(targetTicks) == currentPosition)
        return true;

    if (static_cast<int32_t>(targetTicks) > currentPosition)
        direction = 1;
    else
        direction = -1;

    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        if (m_State.lastDirection != 0 &&
            direction != m_State.lastDirection &&
            m_State.backlashSteps > 0)
        {
            backlashTicksRemaining = m_State.backlashSteps;
        }

        m_State.targetPosition = static_cast<int32_t>(targetTicks);
        m_State.lastDirection = direction;
        m_State.moving = true;
    }

    if (m_MotionThread.joinable())
    {
        m_Abort.store(true, std::memory_order_relaxed);
        m_MotionThread.join();
    }

    DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Motor thread about to start");

    setCurrent(false);
    m_Abort.store(false, std::memory_order_relaxed);
    m_MotionThread = getMotorThread(targetTicks, direction, backlashTicksRemaining);
    return true;
}

bool Focuser::setResolution(int res)
{
    if (res != 1 && res != 2 && res != 4 && res != 8 && res != 16 && res != 32)
        return false;

    int m0 = LOW;
    int m1 = LOW;
    int m2 = LOW;

    switch (res)
    {
    case 1:
        m0 = LOW;
        m1 = LOW;
        m2 = LOW;
        break;
    case 2:
        m0 = HIGH;
        m1 = LOW;
        m2 = LOW;
        break;
    case 4:
        m0 = LOW;
        m1 = HIGH;
        m2 = LOW;
        break;
    case 8:
        m0 = HIGH;
        m1 = HIGH;
        m2 = LOW;
        break;
    case 16:
        m0 = LOW;
        m1 = LOW;
        m2 = HIGH;
        break;
    case 32:
        m0 = HIGH;
        m1 = LOW;
        m2 = HIGH;
        break;
    default:
        return false;
    }

    m_BoardIO.write(M0_PIN, m0);
    m_BoardIO.write(M1_PIN, m1);
    m_BoardIO.write(M2_PIN, m2);

    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.resolution = res;
    return true;
}

bool Focuser::reverseFocuser(bool enabled)
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.reverse = enabled;
    return true;
}

bool Focuser::syncFocuser(uint32_t ticks)
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.currentPosition = clampInt(static_cast<int>(ticks), 0, m_State.maxPosition);
    m_State.targetPosition = m_State.currentPosition;
    return true;
}

bool Focuser::setFocuserBacklash(int32_t steps)
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.backlashSteps = std::max<int32_t>(0, steps);
    return true;
}

bool Focuser::setFocuserMaxPosition(uint32_t ticks)
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.maxPosition = std::max<int32_t>(0, static_cast<int32_t>(ticks));
    if (m_State.currentPosition > m_State.maxPosition)
        m_State.currentPosition = m_State.maxPosition;
    if (m_State.targetPosition > m_State.maxPosition)
        m_State.targetPosition = m_State.maxPosition;
    return true;
}

bool Focuser::setTemperature(double temperatureC)
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.focuserTemperature = temperatureC;
    return true;
}

void Focuser::setRevision(int revision)
{
    m_Revision = revision;
}

bool Focuser::setTemperatureCompensation(bool tempCompEnabled)
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.temperatureCompEnabled = tempCompEnabled;
    return true;
}

bool Focuser::setTemperatureCoefficient(double stepsPerC)
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.temperatureCoefficient = stepsPerC;
    return true;
}

bool Focuser::temperatureCompensation()
{
    double currentTemperature = 0.0;
    double previousTemperature = 0.0;
    double coefficient = 0.0;
    bool enabled = false;

    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        enabled = m_State.temperatureCompEnabled;
        currentTemperature = m_State.focuserTemperature;
        previousTemperature = m_State.lastCompTemperature;
        coefficient = m_State.temperatureCoefficient;
    }

    if (!enabled)
        return false;

    if (currentTemperature < -999.0)
        return false;

    if (previousTemperature < -999.0)
    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        m_State.lastCompTemperature = currentTemperature;
        return true;
    }

    const double delta = currentTemperature - previousTemperature;
    const int correctionSteps = static_cast<int>(delta * coefficient);

    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        m_State.lastCompTemperature = currentTemperature;
    }

    if (correctionSteps == 0)
        return true;

    return moveRelFocuser(correctionSteps);
}

int Focuser::getHoldPower() const
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    return m_State.holdPowerPercent;
}

bool Focuser::setHoldPowerPercent(int percent)
{
    percent = clampInt(percent, 0, 100);

    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        m_State.holdPowerPercent = percent;
    }

    setCurrent(true);
    return true;
}

bool Focuser::setCurrent(int currentmA)
{
    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        m_State.currentmA = clampInt(currentmA, MIN_DRIVER_CURRENT_MA, MAX_DRIVER_CURRENT_MA);
    }

    setCurrent(true);
    return true;
}

void Focuser::setCurrent(bool standby)
{
    int requestedCurrent = 0;
    int holdPercent = 0;

    {
        std::lock_guard<std::mutex> lock(m_StateMutex);
        requestedCurrent = m_State.currentmA;
        holdPercent = m_State.holdPowerPercent;
    }

    if (standby)
    {
        if (m_Revision == 1)
        {
            if (holdPercent == 100)
            {
                m_BoardIO.write(HOLD_PIN, LOW);
                DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Stepper motor enabled 100%%.");
            }
            else if (holdPercent > 0)
            {
                m_BoardIO.write(HOLD_PIN, HIGH);
                DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Stepper motor enabled 50%%.");
            }
            else
            {
                m_BoardIO.write(HOLD_PIN, HIGH);
                DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Stepper motor enabled 0%%.");
            }
        }
        if (m_Revision > 1 && m_Revision < 4)
        {
            m_BoardIO.setDac(m_BoardIO.getConfig().dacChannelRun, 255 * (holdPercent * requestedCurrent / 100) / 4096);
        }
        if (m_Revision >= 4)
        {
            m_PwmController.setDutyPercent(PwmController::Channel::MOT, getMotorPWM(holdPercent * requestedCurrent / 100));
            (holdPercent > 0) ? m_PwmController.enable(PwmController::Channel::MOT) : m_PwmController.disable(PwmController::Channel::MOT);
            DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Current %d hold %d", requestedCurrent, holdPercent);            
        }

        if (holdPercent > 0)
        {
            DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Stepper motor enabled %d %%.", holdPercent);
        }
        else
        {
            DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Stepper motor disabled.");
        }
    }
    else
    {
        m_BoardIO.write(EN_PIN, LOW);
        m_BoardIO.write(DECAY_PIN, HIGH);
        if (m_Revision == 1)
        {
            m_BoardIO.write(HOLD_PIN, LOW);
        }
        if (m_Revision > 1 && m_Revision < 4)
        {
            m_BoardIO.setDac(m_BoardIO.getConfig().dacChannelRun, 255 * requestedCurrent / 4096);
        }
        if (m_Revision >= 4)
        {
            m_PwmController.setDutyPercent(PwmController::Channel::MOT, getMotorPWM(requestedCurrent));
            m_PwmController.enable(PwmController::Channel::MOT);
            DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Current %d hold %d", requestedCurrent, 100);            
        }
    }
}

int Focuser::getMotorPWM(int currentmA) const
{
    // 100 = 1.03V = 2.06A, 1 = 20mA
    return clampInt((currentmA / 20), 0, 100);
}

bool Focuser::setStepDelayUs(int stepDelayUs)
{
    if (stepDelayUs < 50)
        return false;

    std::lock_guard<std::mutex> lock(m_StateMutex);
    m_State.stepDelayUs = stepDelayUs;
    return true;
}

Focuser::State Focuser::getState() const
{
    std::lock_guard<std::mutex> lock(m_StateMutex);
    return m_State;
}

std::thread Focuser::getMotorThread(uint32_t targetPos, int direction, int backlashTicksRemaining)
{
    return std::thread([this, targetPos, direction, backlashTicksRemaining]()
                       {
        int motorDirection = direction;
        int backlashRemaining = backlashTicksRemaining;
        // DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION,  "Inside thread");

        // DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION,  "PIN statuses EN %d HOLD %d DECAY_PIN %d"
        // , m_BoardIO.read(EN_PIN), m_BoardIO.read(HOLD_PIN), m_BoardIO.read(DECAY_PIN));

        while (!m_Abort.load(std::memory_order_relaxed))
        {
            bool reverse = false;
            int stepDelayUs = 0;
            int32_t currentPos = 0;

            {
                std::lock_guard<std::mutex> lock(m_StateMutex);
                reverse = m_State.reverse;
                stepDelayUs = m_State.stepDelayUs;
                currentPos = m_State.currentPosition;
            }

            if (currentPos == static_cast<int32_t>(targetPos))
                break;

            const int dirLevel =
                reverse
                    ? ((motorDirection < 0) ? HIGH : LOW)
                    : ((motorDirection < 0) ? LOW : HIGH);

            m_BoardIO.write(DIR_PIN, dirLevel);
            m_BoardIO.write(STP_PIN, HIGH);
            delayMicroseconds(10);
            m_BoardIO.write(STP_PIN, LOW);
            // DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION,  "STP pin cycled");

            {
                std::lock_guard<std::mutex> lock(m_StateMutex);

                if (backlashRemaining <= 0)
                    m_State.currentPosition += motorDirection;
                else
                    --backlashRemaining;
            }

            delayMicroseconds(stepDelayUs);
        }

        {
            std::lock_guard<std::mutex> lock(m_StateMutex);
            m_State.moving = false;
            m_State.targetPosition = m_State.currentPosition;

        }

        // DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION,  "Motor thread about to end");


        setCurrent(true); });
}

int Focuser::clampInt(int value, int minValue, int maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}