#include "pwm.h"
#include "boardio.h"

#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include <sys/stat.h>

#include <wiringPi.h>
#include <softPwm.h>


PwmController::PwmController(BoardIO &boardIO, const std::string &deviceName)
    : BaseComponent(deviceName, "PWM"), m_BoardIO(boardIO)
{
    m_ChannelStates[Channel::P1] = {};
    m_ChannelStates[Channel::P2] = {};
    m_ChannelStates[Channel::FAN] = {};
    m_ChannelStates[Channel::MOT] = {};
}

PwmController::~PwmController()
{
    shutdown();
}


int PwmController::bcmPin(Channel channel) const
{
    switch (channel)
    {
    case Channel::P1:
        return PWM1_PIN;
    case Channel::P2:
        return PWM2_PIN;
    case Channel::FAN:
        return FAN_PIN;
    case Channel::MOT:
        return MOTOR_PWM;
    }

    return -1;
}

bool PwmController::connect()
{
    if (m_Initialized)
        return true;

    // Lets use always soft PWM first to test
    // if (m_BoardIO.revision() >= 5)
    // {
    //     if (!initializePi5())
    //         return false;

    //     m_Backend = Backend::SysfsPwm;
    // }
    // else
    // {
    if (!initializePi4())
        return false;

    m_Backend = Backend::SoftPwm;
    // }

	setDutyPercent(PwmController::Channel::P1, 0.0);
	setDutyPercent(PwmController::Channel::P2, 0.0);
	setDutyPercent(PwmController::Channel::FAN, 0.0);
	setDutyPercent(PwmController::Channel::MOT, 0.0);

	enable(PwmController::Channel::P1);
	enable(PwmController::Channel::P2);
	// enable(PwmController::Channel::FAN);
	// enable(PwmController::Channel::MOT);    

    m_Initialized = true;
    return true;
}

bool PwmController::updateConfig(const Config& cfg)
{
    Config newConfig = m_Config;

    newConfig.defaultFrequencyHz = cfg.defaultFrequencyHz;
    newConfig.softPwmRange = cfg.softPwmRange;

    for (const auto& [channel, chCfg] : cfg.pi5Channels)
    {
        newConfig.pi5Channels[channel] = chCfg;
    }

    if (newConfig.defaultFrequencyHz == 0)
        return false;

    m_Config = std::move(newConfig);
    return true;
}

void PwmController::shutdown()
{
    if (!m_Initialized)
        return;

    if (m_Backend == Backend::SoftPwm)
    {
        for (const auto &it : m_ChannelStates)
            softPwmWrite(bcmPin(it.first), 0);
    }
    else if (m_Backend == Backend::SysfsPwm)
    {
        for (auto &it : m_Pi5Pwm)
            it.second.disable();

        m_Pi5Pwm.clear();
    }

    m_Backend = Backend::None;
    m_Initialized = false;
}

bool PwmController::isInitialized() const
{
    return m_Initialized;
}

PwmController::Backend PwmController::backend() const
{
    return m_Backend;
}

bool PwmController::setDutyPercent(Channel channel, double dutyPercent)
{
    if (!m_Initialized)
        return false;

    if (dutyPercent < 0.0)
        dutyPercent = 0.0;
    if (dutyPercent > 100.0)
        dutyPercent = 100.0;

    m_ChannelStates[channel].dutyPercent = dutyPercent;

    if (m_Backend == Backend::SoftPwm)
    {
        applyCachedStatePi4(channel);
        return true;
    }

    if (m_Backend == Backend::SysfsPwm)
        return applyCachedStatePi5(channel);

    return false;
}

bool PwmController::setFrequencyHz(Channel channel, uint32_t frequencyHz)
{
    if (!m_Initialized || frequencyHz == 0)
        return false;

    m_ChannelStates[channel].frequencyHz = frequencyHz;

    if (m_Backend == Backend::SoftPwm)
    {
        // softPwm nie daje tu sensownej, niezależnej i precyzyjnej kontroli
        // częstotliwości dla każdego kanału przez prosty interfejs wiringPi.
        // Zostawiamy cache, ale nie próbujemy tego fizycznie przełączać.
        return true;
    }

    if (m_Backend == Backend::SysfsPwm)
        return applyCachedStatePi5(channel);

    return false;
}

bool PwmController::enable(Channel channel)
{
    if (!m_Initialized)
        return false;

    m_ChannelStates[channel].enabled = true;

    if (m_Backend == Backend::SoftPwm)
    {
        applyCachedStatePi4(channel);
        return true;
    }

    if (m_Backend == Backend::SysfsPwm)
        return applyCachedStatePi5(channel);

    return false;
}

bool PwmController::disable(Channel channel)
{
    if (!m_Initialized)
        return false;

    m_ChannelStates[channel].enabled = false;

    if (m_Backend == Backend::SoftPwm)
    {
        softPwmWrite(bcmPin(channel), 0);
        return true;
    }

    if (m_Backend == Backend::SysfsPwm)
        return m_Pi5Pwm[channel].disable();

    return false;
}

double PwmController::getDutyPercent(Channel channel) const
{
    auto it = m_ChannelStates.find(channel);
    if (it == m_ChannelStates.end())
        return 0.0;

    return it->second.dutyPercent;
}

uint32_t PwmController::getFrequencyHz(Channel channel) const
{
    auto it = m_ChannelStates.find(channel);
    if (it == m_ChannelStates.end())
        return 0;

    return it->second.frequencyHz;
}

bool PwmController::initializePi4()
{
    if (wiringPiSetupGpio() < 0)
        return false;

    for (auto &it : m_ChannelStates)
    {
        const int pin = bcmPin(it.first);

        if (softPwmCreate(pin, 0, m_Config.softPwmRange) != 0)
            return false;

        it.second.frequencyHz = m_Config.defaultFrequencyHz;
        it.second.dutyPercent = 0.0;
        it.second.enabled = false;
    }

    return true;
}

bool PwmController::initializePi5()
{
    for (auto &it : m_ChannelStates)
    {
        const auto cfgIt = m_Config.pi5Channels.find(it.first);
        if (cfgIt == m_Config.pi5Channels.end())
            return false;

        SysfsPwm pwm(cfgIt->second.chipPath, cfgIt->second.pwmIndex);
        if (!pwm.open())
            return false;

        m_Pi5Pwm[it.first] = pwm;

        it.second.frequencyHz = m_Config.defaultFrequencyHz;
        it.second.dutyPercent = 0.0;
        it.second.enabled = false;

        if (!applyCachedStatePi5(it.first))
            return false;
    }

    return true;
}

uint64_t PwmController::frequencyToPeriodNs(uint32_t frequencyHz) const
{
    return 1000000000ULL / static_cast<uint64_t>(frequencyHz);
}

void PwmController::applyCachedStatePi4(Channel channel)
{
    const auto &state = m_ChannelStates[channel];
    const int pin = bcmPin(channel);

    if (!state.enabled)
    {
        softPwmWrite(pin, 0);
        return;
    }

    const int value = static_cast<int>((state.dutyPercent / 100.0) * m_Config.softPwmRange + 0.5);
    softPwmWrite(pin, value);
}

bool PwmController::applyCachedStatePi5(Channel channel)
{
    auto pwmIt = m_Pi5Pwm.find(channel);
    if (pwmIt == m_Pi5Pwm.end())
        return false;

    auto &pwm = pwmIt->second;
    const auto &state = m_ChannelStates[channel];

    const uint64_t periodNs = frequencyToPeriodNs(state.frequencyHz);
    const uint64_t dutyNs =
        static_cast<uint64_t>((periodNs * state.dutyPercent) / 100.0);

    if (!pwm.disable())
        return false;

    if (!pwm.setPeriodNs(periodNs))
        return false;

    if (!pwm.setDutyNs(dutyNs))
        return false;

    if (state.enabled)
        return pwm.enable();

    return true;
}

// -------------------- SysfsPwm --------------------

PwmController::SysfsPwm::SysfsPwm(const std::string &chipPath, int pwmIndex)
    : m_ChipPath(chipPath),
      m_PwmIndex(pwmIndex),
      m_PwmPath(chipPath + "/pwm" + std::to_string(pwmIndex))
{
}

PwmController::SysfsPwm::~SysfsPwm()
{
    close();
}

bool PwmController::SysfsPwm::open()
{
    if (m_IsOpen)
        return true;

    if (!exportChannel())
        return false;

    for (int i = 0; i < 20; i++)
    {
        if (pathExists(m_PwmPath))
        {
            m_IsOpen = true;
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return false;
}

void PwmController::SysfsPwm::close()
{
    if (!m_IsOpen)
        return;

    disable();
    unexportChannel();
    m_IsOpen = false;
}

bool PwmController::SysfsPwm::isOpen() const
{
    return m_IsOpen;
}

bool PwmController::SysfsPwm::setPeriodNs(uint64_t periodNs)
{
    return writeFile(m_PwmPath + "/period", std::to_string(periodNs));
}

bool PwmController::SysfsPwm::setDutyNs(uint64_t dutyNs)
{
    return writeFile(m_PwmPath + "/duty_cycle", std::to_string(dutyNs));
}

bool PwmController::SysfsPwm::enable()
{
    return writeFile(m_PwmPath + "/enable", "1");
}

bool PwmController::SysfsPwm::disable()
{
    return writeFile(m_PwmPath + "/enable", "0");
}

bool PwmController::SysfsPwm::exportChannel()
{
    return writeFile(m_ChipPath + "/export", std::to_string(m_PwmIndex));
}

bool PwmController::SysfsPwm::unexportChannel()
{
    return writeFile(m_ChipPath + "/unexport", std::to_string(m_PwmIndex));
}

bool PwmController::SysfsPwm::writeFile(const std::string &path, const std::string &value) const
{
    std::ofstream out(path);
    if (!out)
        return false;

    out << value;
    return static_cast<bool>(out);
}

bool PwmController::SysfsPwm::pathExists(const std::string &path) const
{
    struct stat st{};
    return stat(path.c_str(), &st) == 0;
}