#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

class I2CBus
{
public:
    explicit I2CBus(uint8_t deviceAddress);
    ~I2CBus();

    bool open(int busNumber = 1);
    bool isOpen() const;
    void close();

    int write(const char* data, std::size_t length);
    int read(char* buffer, std::size_t length);

    int writeRegister(uint8_t reg, const char* data, std::size_t length);
    int readRegister(uint8_t reg, char* buffer, std::size_t length);

    int writeRegisterByte(uint8_t reg, uint8_t value);
    int readRegisterByte(uint8_t reg, uint8_t& value);

    int readWordData(uint8_t reg, uint16_t &value);

    uint8_t getDeviceAddress() const;
    std::string getLastError() const;

private:
    int fd_;
    uint8_t deviceAddress_;
    std::string devicePath_;
    std::string lastError_;

    void setError(const std::string& msg);
};