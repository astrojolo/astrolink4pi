#include "i2cbus.h"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdint>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <vector>

I2CBus::I2CBus(uint8_t deviceAddress)
    : fd_(-1),
      deviceAddress_(deviceAddress)
{
}

I2CBus::~I2CBus()
{
    close();
}

bool I2CBus::open(int busNumber)
{
    if (isOpen())
        return true;

    devicePath_ = "/dev/i2c-" + std::to_string(busNumber);

    fd_ = ::open(devicePath_.c_str(), O_RDWR);
    if (fd_ < 0)
    {
        setError("Nie można otworzyć " + devicePath_ + ": " + std::string(std::strerror(errno)));
        return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, deviceAddress_) < 0)
    {
        std::ostringstream oss;
        oss << "Nie można ustawić adresu I2C 0x"
            << std::hex << std::uppercase << static_cast<int>(deviceAddress_)
            << ": " << std::strerror(errno);

        setError(oss.str());
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    lastError_.clear();
    return true;
}

bool I2CBus::isOpen() const
{
    return fd_ >= 0;
}

void I2CBus::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

int I2CBus::write(const char *data, std::size_t length)
{
    if (!isOpen())
    {
        setError("Magistrala I2C nie jest otwarta");
        return -1;
    }

    if (data == nullptr || length == 0)
    {
        setError("Nieprawidłowe dane do zapisu");
        return -1;
    }

    const ssize_t written = ::write(fd_, data, length);
    if (written < 0)
    {
        setError("Błąd zapisu I2C: " + std::string(std::strerror(errno)));
        return -1;
    }

    return static_cast<int>(written);
}

int I2CBus::read(char *buffer, std::size_t length)
{
    if (!isOpen())
    {
        setError("Magistrala I2C nie jest otwarta");
        return -1;
    }

    if (buffer == nullptr || length == 0)
    {
        setError("Nieprawidłowy bufor odczytu");
        return -1;
    }

    const ssize_t received = ::read(fd_, buffer, length);
    if (received < 0)
    {
        setError("Błąd odczytu I2C: " + std::string(std::strerror(errno)));
        return -1;
    }

    return static_cast<int>(received);
}

int I2CBus::writeRegister(uint8_t reg, const char *data, std::size_t length)
{
    if (!isOpen())
    {
        setError("Magistrala I2C nie jest otwarta");
        return -1;
    }

    if (data == nullptr || length == 0)
    {
        setError("Nieprawidłowe dane do zapisu rejestru");
        return -1;
    }

    std::vector<char> tx(length + 1);
    tx[0] = static_cast<char>(reg);
    std::memcpy(&tx[1], data, length);

    const ssize_t written = ::write(fd_, tx.data(), tx.size());
    if (written < 0)
    {
        setError("Błąd zapisu do rejestru I2C: " + std::string(std::strerror(errno)));
        return -1;
    }

    // Zwracamy liczbę bajtów danych użytkownika, bez bajtu adresu rejestru.
    if (written == 0)
        return 0;

    return static_cast<int>(written - 1);
}

int I2CBus::readRegister(uint8_t reg, char *buffer, std::size_t length)
{
    if (!isOpen())
    {
        setError("Magistrala I2C nie jest otwarta");
        return -1;
    }

    if (buffer == nullptr || length == 0)
    {
        setError("Nieprawidłowy bufor odczytu rejestru");
        return -1;
    }

    // Najpierw ustawiamy wskaźnik rejestru
    const char regByte = static_cast<char>(reg);
    const ssize_t regWritten = ::write(fd_, &regByte, 1);
    if (regWritten < 0)
    {
        setError("Błąd ustawiania adresu rejestru I2C: " + std::string(std::strerror(errno)));
        return -1;
    }

    if (regWritten != 1)
    {
        setError("Nie udało się wysłać adresu rejestru I2C");
        return -1;
    }

    const ssize_t received = ::read(fd_, buffer, length);
    if (received < 0)
    {
        setError("Błąd odczytu rejestru I2C: " + std::string(std::strerror(errno)));
        return -1;
    }

    return static_cast<int>(received);
}

int I2CBus::writeRegisterByte(uint8_t reg, uint8_t value)
{
    const char data = static_cast<char>(value);
    return writeRegister(reg, &data, 1);
}

int I2CBus::readRegisterByte(uint8_t reg, uint8_t &value)
{
    char data = 0;
    const int result = readRegister(reg, &data, 1);
    if (result == 1)
    {
        value = static_cast<uint8_t>(data);
    }
    return result;
}

int I2CBus::readRegisterTransaction(uint8_t reg, char *buffer, std::size_t length)
{
    if (!isOpen())
    {
        setError("Magistrala I2C nie jest otwarta");
        return -1;
    }

    if (buffer == nullptr || length == 0)
    {
        setError("Nieprawidłowy bufor odczytu rejestru");
        return -1;
    }

    uint8_t regBuf = reg;

    struct i2c_msg messages[2];
    messages[0].addr = address_;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &regBuf;

    messages[1].addr = address_;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<__u16>(length);
    messages[1].buf = reinterpret_cast<uint8_t *>(buffer);

    struct i2c_rdwr_ioctl_data ioctlData;
    ioctlData.msgs = messages;
    ioctlData.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &ioctlData) < 0)
    {
        setError("Błąd transakcji I2C_RDWR: " + std::string(std::strerror(errno)));
        return -1;
    }

    return static_cast<int>(length);
}

uint8_t I2CBus::getDeviceAddress() const
{
    return deviceAddress_;
}

std::string I2CBus::getLastError() const
{
    return lastError_;
}

void I2CBus::setError(const std::string &msg)
{
    lastError_ = msg;
}