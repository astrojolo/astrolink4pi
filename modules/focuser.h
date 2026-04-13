#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "basecomponent.h"
#include "boardio.h"
#include "pwm.h"

class Focuser : public BaseComponent
{
public:
    struct Config
    {
        int pinEN = 15;    // pin 10
        int pinM0 = 17;    // pin 11
        int pinM1 = 18;    // pin 12
        int pinM2 = 27;    // pin 13
        int pinRST = 22;   // pin 15
        int pinSTP = 24;   // pin 18
        int pinDIR = 23;   // pin 16
        int pinDecay = 14; // pin 8
        int pinHold = 10;  // pin 19 EN

        int maxResolution = 32;
        int defaultResolution = 1;
        int defaultStepDelayUs = 2000;
        int defaultCurrentmA = 600;
        int defaultMaxPosition = 100000;
    };

    struct State
    {
        bool connected = false;
        bool moving = false;
        bool reverse = false;
        bool temperatureCompEnabled = false;

        int resolution = 1;
        int holdPowerPercent = 0;
        int stepDelayUs = 2000;
        int currentmA = 600;

        int32_t currentPosition = 0;
        int32_t targetPosition = 0;
        int32_t maxPosition = 100000;
        int32_t backlashSteps = 0;
        int32_t lastDirection = 0;

        double focuserTemperature = -1000.0;
        double lastCompTemperature = -1000.0;
        double temperatureCoefficient = 0.0;
    };

    Focuser(const Config &config, BoardIO &boardIO, PwmController &pwmController, const std::string &deviceName);
    ~Focuser();

    bool open();
    void close();
    bool isOpen() const;

    bool abortFocuser();
    bool moveRelFocuser(int32_t ticks);
    bool moveAbsFocuser(uint32_t targetTicks);

    bool setResolution(int res);
    bool reverseFocuser(bool enabled);
    bool syncFocuser(uint32_t ticks);
    bool setFocuserBacklash(int32_t steps);
    bool setFocuserMaxPosition(uint32_t ticks);

    bool setTemperature(double temperatureC);
    bool setTemperatureCompensation(bool tempCompEnabled);
    bool setTemperatureCoefficient(double stepsPerC);
    bool temperatureCompensation();

    int getHoldPower() const;
    bool setHoldPowerPercent(int percent);

    bool setCurrent(int currentmA);
    void setCurrent(bool standby);
    int getMotorPWM(int currentmA) const;
    int setDac(int chan, int value);

    void setRevision(int revision);

    bool setStepDelayUs(int stepDelayUs);

    State getState() const;

private:
    std::thread getMotorThread(uint32_t targetPos, int direction, int backlashTicksRemaining);

    static int clampInt(int value, int minValue, int maxValue);

private:
    BoardIO &m_BoardIO;
    PwmController &m_PwmController;
    Config m_Config;
    mutable std::mutex m_StateMutex;
    State m_State;

    std::thread m_MotionThread;
    std::atomic<bool> m_Abort{false};

    int m_Revision = 0;
};