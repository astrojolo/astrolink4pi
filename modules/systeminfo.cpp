#include "systeminfo.h"

#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <array>

SystemInfoService::SystemInfoService() = default;

SystemInfoService::~SystemInfoService() = default;

std::string SystemInfoService::getHostname() const
{
    return trim(readFile("/etc/hostname"));
}

std::string SystemInfoService::getModel() const
{
    return trim(readFile("/proc/device-tree/model"));
}

std::string SystemInfoService::getKernelVersion() const
{
    return trim(runCommand("uname -r"));
}

std::string SystemInfoService::getCpuTemp() const
{
    return trim(runCommand("echo $(($(cat /sys/class/thermal/thermal_zone0/temp)/1000))"));
}

std::string SystemInfoService::getLoad() const
{
    return trim(runCommand("uptime|awk -F, '{print $3\" /\"$4\" /\"$5}'|awk -F: '{print $2}'|xargs"));
}

std::string SystemInfoService::getLocalIP() const
{
    return trim(runCommand("hostname -I|awk -F' '  '{print $1}'|xargs"));
}

std::string SystemInfoService::getPublicIP() const
{
    return trim(runCommand("curl -s ifconfig.me"));
}

std::string SystemInfoService::getUptimeString() const
{
    std::ifstream in("/proc/uptime");
    if (!in)
        return {};

    double uptimeSeconds = 0.0;
    in >> uptimeSeconds;

    const uint64_t totalSeconds = static_cast<uint64_t>(uptimeSeconds);
    const uint64_t days = totalSeconds / 86400;
    const uint64_t hours = (totalSeconds % 86400) / 3600;
    const uint64_t minutes = (totalSeconds % 3600) / 60;
    const uint64_t seconds = totalSeconds % 60;

    std::ostringstream ss;

    if (days > 0)
        ss << days << "d ";

    if (hours > 0 || days > 0)
        ss << hours << "h ";

    if (minutes > 0 || hours > 0 || days > 0)
        ss << minutes << "m ";

    ss << seconds << "s";

    return ss.str();
}

uint64_t SystemInfoService::millis() const
{
    using namespace std::chrono;

    return duration_cast<milliseconds>(
               steady_clock::now().time_since_epoch())
        .count();
}

std::string SystemInfoService::runCommand(const std::string &command) const
{
    std::array<char, 256> buffer {};
    std::string result;

    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
        return {};

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr)
        result += buffer.data();

    pclose(pipe);

    return trim(result);
}

std::string SystemInfoService::readFile(const std::string &path) const
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
        return {};

    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string SystemInfoService::trim(const std::string &value) const
{
    const auto first = value.find_first_not_of(" \t\r\n\0", 0);
    if (first == std::string::npos)
        return {};

    const auto last = value.find_last_not_of(" \t\r\n\0");
    return value.substr(first, last - first + 1);
}