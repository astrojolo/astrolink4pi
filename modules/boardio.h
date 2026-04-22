#ifndef BOARDIO_H
#define BOARDIO_H

#include <string>


#include "basecomponent.h"


class BoardIO : public BaseComponent
{
public:
    static constexpr int RP4_GPIOCHIP = 4;
    static constexpr int RP5_GPIOCHIP = 5;
    static constexpr int RP_UNKNOWN = 0;

    static constexpr int OUT1_PIN = 5; // pin 29
    static constexpr int OUT2_PIN = 6; // pin 31
    static constexpr int MOTOR_PWM = 20;		// pin 38 VOUT
    static constexpr int CHK_IN_PIN = 16;		// pin 36
    static constexpr int CHK2_IN_PIN = 21;		// pin 40    

    struct Config
    {
        int spiChannel = 1;
        int spiSpeed = 1000000;

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
  
    bool setOut1(int value);
    bool setOut2(int value);

    void write(int gpio, int value);
    int read(int gpio) const;
    void initializePin(int gpio, int mode, int value);
    int setDac(int chan, int value);
    int setDacRun(int value);
    int setDacHold(int value);
    
    static bool writeDac(uint8_t channel,
                         uint8_t value,
                         bool gain1x = true,
                         bool active = true,
                         const char* device = "/dev/spidev0.1",
                         uint32_t speedHz = 1000000);    

private:
    int detectBoard();
    int checkRevision();
    std::string readFile(const std::string &path) const;

private:
    Config m_Config;

    bool m_Connected = false;
    int m_Revision = 0;
    int m_GpioChip = RP_UNKNOWN;
};

#endif