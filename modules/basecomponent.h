#pragma once

#include <string>
#include <cstdarg>
#include <cstdio>
#include <indilogger.h>

class BaseComponent
{
public:
    BaseComponent(const std::string &deviceName, const std::string &componentName)
        : m_DeviceName(deviceName), m_ComponentName(componentName)
    {
    }

    virtual ~BaseComponent() = default;

protected:
    void logInfo(const char *format, ...) const
    {
        va_list args;
        va_start(args, format);
        logMessage(INDI::Logger::DBG_SESSION, format, args);
        va_end(args);
    }

    void logDebug(const char *format, ...) const
    {
        va_list args;
        va_start(args, format);
        logMessage(INDI::Logger::DBG_DEBUG, format, args);
        va_end(args);
    }

    void logWarn(const char *format, ...) const
    {
        va_list args;
        va_start(args, format);
        logMessage(INDI::Logger::DBG_WARNING, format, args);
        va_end(args);
    }

    void logError(const char *format, ...) const
    {
        va_list args;
        va_start(args, format);
        logMessage(INDI::Logger::DBG_ERROR, format, args);
        va_end(args);
    }

    const std::string &getDeviceName() const { return m_DeviceName; }
    const std::string &getComponentName() const { return m_ComponentName; }

private:
    std::string m_DeviceName;
    std::string m_ComponentName;

    void logMessage(INDI::Logger::VerbosityLevel level,
                    const char *format,
                    va_list args) const
    {
        char userMessage[1024];
        vsnprintf(userMessage, sizeof(userMessage), format, args);

        char finalMessage[1280];
        snprintf(finalMessage, sizeof(finalMessage), "[%s][%s] %s",
                 m_DeviceName.c_str(),
                 m_ComponentName.c_str(),
                 userMessage);

        INDI::Logger::getInstance().print(level, nullptr, finalMessage);
    }
};