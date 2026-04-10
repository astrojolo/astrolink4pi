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
    const std::string &getDeviceName() const { return m_DeviceName; }
    const std::string &getComponentName() const { return m_ComponentName; }

private:
    std::string m_DeviceName;
    std::string m_ComponentName;

};