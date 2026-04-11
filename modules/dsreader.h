#pragma once

#include <string>

#include "basecomponent.h"

class DSFileReader : public BaseComponent
{
public:
    struct Readings
    {
        double temperature = 0.0;
    };

    DSFileReader(const std::string &devicePath, const std::string &deviceName);
    ~DSFileReader();

    bool open();
    void close();
    bool isOpen() const;
    bool read(Readings &out);

private:
    bool readRawFile(std::string &content) const;
    bool parseTemperature(const std::string &content, double &temperature) const;

private:
    std::string m_DevicePath;
    bool m_IsOpen = false;
    Readings m_LastReadings;
};