#pragma once

#include <array>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>

#include <wiringPi.h>
#include <softPwm.h>

class AstroLinkPwm
{
public:
    enum class Channel
    {
        P1,
        P2,
        FAN,
        MOT
    };

    enum class Board
    {
        Unknown,
        RPi4,
        RPi5
    };

    struct Pi5PwmConfig
    {
        // Ścieżki trzeba dopasować do konkretnego systemu po restarcie,
        // np. /sys/class/pwm/pwmchip0, pwmchip1, ...
        std::map<Channel, std::string> chipPathByChannel;
        uint32_t frequencyHz = 25000; // przykładowo dla FAN/PWM
    };

    AstroLinkPwm(const Pi5PwmConfig& pi5Config)
        : board_(detectBoard()), pi5Config_(pi5Config)
    {
        if (board_ == Board::RPi4)
        {
            initPi4();
        }
        else if (board_ == Board::RPi5)
        {
            initPi5();
        }
        else
        {
            throw std::runtime_error("Unsupported Raspberry Pi model");
        }
    }

    ~AstroLinkPwm()
    {
        try
        {
            shutdownAll();
        }
        catch (...)
        {
        }
    }

    Board board() const
    {
        return board_;
    }

    void setDutyPercent(Channel ch, double dutyPercent)
    {
        if (dutyPercent < 0.0)
            dutyPercent = 0.0;
        if (dutyPercent > 100.0)
            dutyPercent = 100.0;

        if (board_ == Board::RPi4)
        {
            int pin = bcmPin(ch);
            softPwmWrite(pin, static_cast<int>(dutyPercent + 0.5));
        }
        else if (board_ == Board::RPi5)
        {
            auto& pwm = pi5Pwm_.at(ch);
            uint64_t dutyNs = static_cast<uint64_t>(
                (pwm.periodNs * dutyPercent) / 100.0
            );
            pwm.setDutyNs(dutyNs);
        }
    }

    void setFrequencyHz(Channel ch, uint32_t frequencyHz)
    {
        if (frequencyHz == 0)
            throw std::runtime_error("frequencyHz must be > 0");

        if (board_ == Board::RPi4)
        {
            // softPwm z wiringPi nie daje wygodnej, precyzyjnej kontroli częstotliwości
            // per kanał. Tu zostawiamy stały softPwm range=100.
            // Możesz dodać własny soft PWM jeśli potrzebujesz innych częstotliwości.
            (void)ch;
            (void)frequencyHz;
        }
        else if (board_ == Board::RPi5)
        {
            auto& pwm = pi5Pwm_.at(ch);
            pwm.disable();
            pwm.periodNs = 1'000'000'000ULL / frequencyHz;
            pwm.setPeriodNs(pwm.periodNs);
            pwm.enable();
        }
    }

    void disable(Channel ch)
    {
        if (board_ == Board::RPi4)
        {
            softPwmWrite(bcmPin(ch), 0);
        }
        else if (board_ == Board::RPi5)
        {
            pi5Pwm_.at(ch).disable();
        }
    }

private:
    class SysfsPwm
    {
    public:
        SysfsPwm() = default;

        SysfsPwm(const std::string& chipPath, int channel, uint64_t periodNs)
            : chipPath_(chipPath),
              channel_(channel),
              pwmPath_(chipPath + "/pwm" + std::to_string(channel)),
              periodNs(periodNs)
        {
            exportChannel();
            disable();
            setPeriodNs(periodNs);
            setDutyNs(0);
            enable();
        }

        ~SysfsPwm()
        {
            try
            {
                disable();
                unexportChannel();
            }
            catch (...)
            {
            }
        }

        void setPeriodNs(uint64_t ns)
        {
            writeFile(pwmPath_ + "/period", std::to_string(ns));
        }

        void setDutyNs(uint64_t ns)
        {
            if (ns > periodNs)
                ns = periodNs;
            writeFile(pwmPath_ + "/duty_cycle", std::to_string(ns));
        }

        void enable()
        {
            writeFile(pwmPath_ + "/enable", "1");
        }

        void disable()
        {
            writeFile(pwmPath_ + "/enable", "0");
        }

        uint64_t periodNs = 0;

    private:
        void exportChannel()
        {
            try
            {
                writeFile(chipPath_ + "/export", std::to_string(channel_));
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            catch (...)
            {
                // kanał może już być wyeksportowany
            }
        }

        void unexportChannel()
        {
            try
            {
                writeFile(chipPath_ + "/unexport", std::to_string(channel_));
            }
            catch (...)
            {
            }
        }

        static void writeFile(const std::string& path, const std::string& value)
        {
            std::ofstream f(path);
            if (!f)
                throw std::runtime_error("Cannot open " + path);
            f << value;
            if (!f)
                throw std::runtime_error("Cannot write " + path);
        }

        std::string chipPath_;
        int channel_ = 0;
        std::string pwmPath_;
    };

    static Board detectBoard()
    {
        std::ifstream f("/proc/device-tree/model", std::ios::binary);
        if (!f)
            return Board::Unknown;

        std::string model((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());

        if (model.find("Raspberry Pi 5") != std::string::npos)
            return Board::RPi5;
        if (model.find("Raspberry Pi 4") != std::string::npos)
            return Board::RPi4;

        return Board::Unknown;
    }

    static int bcmPin(Channel ch)
    {
        switch (ch)
        {
            case Channel::P1:  return 19;
            case Channel::P2:  return 26;
            case Channel::FAN: return 13;
            case Channel::MOT: return 20;
        }
        throw std::runtime_error("Invalid channel");
    }

    void initPi4()
    {
        if (wiringPiSetupGpio() < 0)
            throw std::runtime_error("wiringPiSetupGpio failed");

        for (Channel ch : allChannels())
        {
            int pin = bcmPin(ch);
            if (softPwmCreate(pin, 0, 100) != 0)
                throw std::runtime_error("softPwmCreate failed on GPIO " + std::to_string(pin));
        }
    }

    void initPi5()
    {
        const uint64_t periodNs = 1'000'000'000ULL / pi5Config_.frequencyHz;

        for (Channel ch : allChannels())
        {
            auto it = pi5Config_.chipPathByChannel.find(ch);
            if (it == pi5Config_.chipPathByChannel.end())
                throw std::runtime_error("Missing pwmchip path for channel");

            // dla pwm-pio zwykle kanał w danym chipie będzie 0
            pi5Pwm_.emplace(ch, SysfsPwm(it->second, 0, periodNs));
        }
    }

    void shutdownAll()
    {
        for (Channel ch : allChannels())
        {
            if (board_ == Board::RPi4)
            {
                softPwmWrite(bcmPin(ch), 0);
            }
            else if (board_ == Board::RPi5)
            {
                auto it = pi5Pwm_.find(ch);
                if (it != pi5Pwm_.end())
                    it->second.disable();
            }
        }
    }

    static std::array<Channel, 4> allChannels()
    {
        return { Channel::P1, Channel::P2, Channel::FAN, Channel::MOT };
    }

    Board board_ = Board::Unknown;
    Pi5PwmConfig pi5Config_;
    std::map<Channel, SysfsPwm> pi5Pwm_;
};