#ifndef BOARDIO_H
#define BOARDIO_H

#include <string>
#include <cstdint>
#include <wiringPi.h>

#include "basecomponent.h"

class BoardIO : public BaseComponent
{
public:
    static constexpr int RP4_GPIOCHIP = 4;
    static constexpr int RP5_GPIOCHIP = 5;

    BoardIO(const std::string &deviceName);
    ~BoardIO();

    bool connect();
    void disconnect();
    bool isConnected() const;

    int revision() const;
    int gpioChip() const;
    int handle() const;

    void write(int gpio, int value);
    int read(int gpio) const;
    void initializePin(int gpio, int mode, int value);
    int setDac(int chan, int value);

private:
    int detectBoard();
    int checkRevision();
    std::string readFile(const std::string &path) const;

private:
    int m_SpiFd = -1;

    int m_Revision = 0;
    int m_GpioChip = RP4_GPIOCHIP;
};

#endif