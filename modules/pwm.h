#ifndef PWMCONTROLLER_H
#define PWMCONTROLLER_H

#include <cstdint>
#include <map>
#include <string>

#include "basecomponent.h"
#include "boardio.h"

// dtoverlay=pwm-pio,gpio=19
// dtoverlay=pwm-pio,gpio=26
// dtoverlay=pwm-pio,gpio=13
// dtoverlay=pwm-pio,gpio=20

// pwmConfig.pi5Channels[PwmController::Channel::P1]  = {"/sys/class/pwm/pwmchip0", 0}; // GPIO19
// pwmConfig.pi5Channels[PwmController::Channel::P2]  = {"/sys/class/pwm/pwmchip1", 0}; // GPIO26
// pwmConfig.pi5Channels[PwmController::Channel::FAN] = {"/sys/class/pwm/pwmchip2", 0}; // GPIO13
// pwmConfig.pi5Channels[PwmController::Channel::MOT] = {"/sys/class/pwm/pwmchip3", 0}; // GPIO20

class PwmController : public BaseComponent
{
public:
    static constexpr int PWM1_PIN = 26;  // pin 37
    static constexpr int PWM2_PIN = 19;  // pin 35
    static constexpr int MOTOR_PWM = 20; // pin 38 VOUT
    static constexpr int FAN_PIN = 13;   // pin 33

    enum class Channel
    {
        P1,
        P2,
        FAN,
        MOT
    };

    enum class Backend
    {
        None,
        SoftPwm,
        SysfsPwm
    };

    struct Pi5ChannelConfig
    {
        std::string chipPath; // np. /sys/class/pwm/pwmchip0
        int pwmIndex = 0;     // zwykle 0 dla pwm-pio
    };

    struct Config
    {
        uint32_t defaultFrequencyHz = 1000;
        std::map<Channel, int> softPwmRanges = {
            {Channel::P1, 100},
            {Channel::P2, 100},
            {Channel::FAN, 20},
            {Channel::MOT, 20}
        };

        std::map<Channel, Pi5ChannelConfig> pi5Channels;
    };

    explicit PwmController(BoardIO &boardIO, const std::string &deviceName);
    ~PwmController();

    bool connect();
    bool updateConfig(const Config& cfg);

    void shutdown();

    bool isInitialized() const;
    Backend backend() const;

    bool setDutyPercent(Channel channel, double dutyPercent);
    bool setFrequencyHz(Channel channel, uint32_t frequencyHz);
    bool enable(Channel channel);
    bool disable(Channel channel);

    double getDutyPercent(Channel channel) const;
    uint32_t getFrequencyHz(Channel channel) const;

private:
    struct ChannelState
    {
        double dutyPercent = 0.0;
        uint32_t frequencyHz = 1000;
        bool enabled = false;
    };

    class SysfsPwm
    {
    public:
        SysfsPwm() = default;
        SysfsPwm(const std::string &chipPath, int pwmIndex);
        ~SysfsPwm();

        bool open();
        void close();

        bool isOpen() const;

        bool setPeriodNs(uint64_t periodNs);
        bool setDutyNs(uint64_t dutyNs);
        bool enable();
        bool disable();

    private:
        bool exportChannel();
        bool unexportChannel();
        bool writeFile(const std::string &path, const std::string &value) const;
        bool pathExists(const std::string &path) const;

    private:
        std::string m_ChipPath;
        int m_PwmIndex = 0;
        std::string m_PwmPath;
        bool m_IsOpen = false;
    };

private:
    bool initializePi4();
    bool initializePi5();

    int bcmPin(Channel channel) const;
    uint64_t frequencyToPeriodNs(uint32_t frequencyHz) const;
    void applyCachedStatePi4(Channel channel);
    bool applyCachedStatePi5(Channel channel);

    int softPwmRange(Channel channel) const;

private:
    BoardIO &m_BoardIO;
    Backend m_Backend = Backend::None;
    bool m_Initialized = false;
    Config m_Config;

    std::map<Channel, ChannelState> m_ChannelStates;
    std::map<Channel, SysfsPwm> m_Pi5Pwm;
};

#endif