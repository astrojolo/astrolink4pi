#ifndef SYSTEMINFOSERVICE_H
#define SYSTEMINFOSERVICE_H

#include <string>
#include <cstdint>

#include "basecomponent.h"

class SystemInfoService : public BaseComponent
{
public:
    static constexpr int SYSTEM_UPDATE_PERIOD = 1000;

    SystemInfoService(const std::string &deviceName);
    ~SystemInfoService();

    void update();

    std::string getHostname() const;
    std::string getModel() const;
    std::string getKernelVersion() const;
    std::string getUptimeString() const;
    std::string getCpuTemp() const;
    std::string getLoad() const;
    std::string getLocalIP() const;
    std::string getPublicIP() const;

    uint64_t millis() const;

    std::string runCommand(const std::string &command) const;
    std::string readFile(const std::string &path) const;

private:
    std::string trim(const std::string &value) const;
    uint64_t nextSystemRead = 0;
};

#endif