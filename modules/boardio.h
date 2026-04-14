#ifndef BOARDIO_H
#define BOARDIO_H

#include <string>
#include <cstdint>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "basecomponent.h"

class BoardIO : public BaseComponent
{
public:
    static constexpr int RP4_GPIOCHIP = 4;
    static constexpr int RP5_GPIOCHIP = 5;
    static constexpr int OUT1_PIN = 5; // pin 29
    static constexpr int OUT2_PIN = 6; // pin 31

    struct Config
    {
        int spiChannel = 0;
        int spiSpeed = 500000;

        int dacChannelRun = 0;
        int dacChannelHold = 1;
    };

    BoardIO(const std::string &deviceName);
    ~BoardIO();

    bool connect();
    void disconnect();
    bool isConnected() const;

    int revision() const;
    int gpioChip() const;
    int handle() const;

    bool setOut1(int value);
    bool setOut2(int value);

    void write(int gpio, int value);
    int read(int gpio) const;
    void initializePin(int gpio, int mode, int value);
    int setDac(int chan, int value);
    Config getConfig() const;

private:
    int detectBoard();
    int checkRevision();
    std::string readFile(const std::string &path) const;

private:
    Config m_Config;
    int m_SpiFd = -1;

    int m_Revision = 0;
    int m_GpioChip = RP4_GPIOCHIP;
};

#endif