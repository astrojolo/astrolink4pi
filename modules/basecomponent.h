#pragma once

#include <algorithm>
#include <string>

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
    std::string getFullName() const
    {
        return m_DeviceName + ":" + m_ComponentName;
    }

    static int clampInt(int value, int minValue, int maxValue)
    {
        return std::max(minValue, std::min(value, maxValue));
    }    

private:
    std::string m_DeviceName;
    std::string m_ComponentName;
};