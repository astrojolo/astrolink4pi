#pragma once

#include <cstdint>
#include <string>
#include <vector>

class I2CBus
{
public:
    I2CBus(std::string devicePath, uint8_t address);
    ~I2CBus();

    bool openBus();
    void closeBus();
    bool isOpen() const;

    bool writeBytes(const uint8_t *data, size_t len);
    bool readBytes(uint8_t *data, size_t len);

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);

    bool readRegister16(uint8_t reg, uint16_t &value, bool swapBytes = true);

    int fd() const { return m_Fd; }
    uint8_t address() const { return m_Address; }

private:
    bool selectSlave();

    std::string m_DevicePath;
    uint8_t m_Address = 0;
    int m_Fd = -1;
};