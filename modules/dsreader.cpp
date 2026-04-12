#include "dsreader.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

DSFileReader::DSFileReader(const std::string &devicePath, const std::string &deviceName)
    : BaseComponent(deviceName, "DSFileReader"), m_DevicePath(devicePath)
{
}

DSFileReader::~DSFileReader()
{
    close();
}

bool DSFileReader::open()
{
    close();

    if (m_DevicePath.empty())
    {
        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "DS file path is empty");
        return false;
    }

    FILE *fp = fopen(m_DevicePath.c_str(), "r");
    if (fp == nullptr)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
                     "Cannot open DS file %s: errno=%d (%s)",
                     m_DevicePath.c_str(), errno, std::strerror(errno));
        return false;
    }

    fclose(fp);
    m_IsOpen = true;
    return true;
}

void DSFileReader::close()
{
    m_IsOpen = false;
}

bool DSFileReader::isOpen() const
{
    return m_IsOpen;
}

bool DSFileReader::read(DSFileReader::Readings &out)
{
    if (!isOpen())
        return false;

    std::string content;
    if (!readRawFile(content))
        return false;

    double temperature = 0.0;
    if (!parseTemperature(content, temperature))
        return false;

    m_LastReadings.temperature = temperature;
    out = m_LastReadings;

    DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION,
                 "DS file read temp=%g C", temperature);

    return true;
}

bool DSFileReader::readRawFile(std::string &content) const
{
    FILE *fp = fopen(m_DevicePath.c_str(), "r");
    if (fp == nullptr)
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "Cannot open DS file %s: errno=%d (%s)",
                     m_DevicePath.c_str(), errno, std::strerror(errno));
        return false;
    }

    char buffer[256];
    content.clear();

    while (fgets(buffer, sizeof(buffer), fp) != nullptr)
        content += buffer;

    if (ferror(fp))
    {
        DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "Error while reading DS file %s",
                     m_DevicePath.c_str());
        fclose(fp);
        return false;
    }

    fclose(fp);
    return !content.empty();
}

bool DSFileReader::parseTemperature(const std::string &content, double &temperature) const
{
    // Wymagamy poprawnego CRC reported by kernel driver: "... YES"
    if (content.find("YES") == std::string::npos)
    {
        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "DS file content has no valid CRC flag (YES)");
        return false;
    }

    const std::string key = "t=";
    const std::size_t pos = content.find(key);
    if (pos == std::string::npos)
    {
        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "DS file content has no temperature marker");
        return false;
    }

    const char *start = content.c_str() + pos + key.size();
    char *end = nullptr;

    errno = 0;
    long value = std::strtol(start, &end, 10);
    if (start == end || errno != 0)
    {
        DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
                     "Cannot parse DS temperature value");
        return false;
    }

    temperature = static_cast<double>(value) / 1000.0;
    return true;
}