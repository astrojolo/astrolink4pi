#include "boardio.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <cstring>
#include <cerrno>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp4802.h>

namespace
{
	constexpr int DAC_MAX_VALUE = 255;
	constexpr int DAC_MIN_VALUE = 0;
}

BoardIO::BoardIO(const std::string &deviceName)
	: BaseComponent(deviceName, "BoardIO")
{
}

BoardIO::~BoardIO()
{
	disconnect();
}

bool BoardIO::connect()
{
	if (isConnected())
		return true;

	int wiringPiSetup = wiringPiSetupGpio();
	if (wiringPiSetup < 0)
	{
		return false;
	}
	m_Connected = true;

	initializePin(MOTOR_PWM, INPUT, LOW);  // pin38
	while (true)
	{
		setDac(0,255);
		setDac(1,200);
		delay(3000);
		setDac(0,0);
		setDac(1,0);
		delay(3000);
	}

	initializePin(OUT1_PIN, OUTPUT, HIGH);
	initializePin(OUT2_PIN, OUTPUT, LOW);

	m_GpioChip = detectBoard();
	DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "Checking rev");
	m_Revision = checkRevision();

	return true;
}

void BoardIO::disconnect()
{
	m_Revision = 0;
	m_GpioChip = RP_UNKNOWN;
	m_Connected = false;
}

bool BoardIO::isConnected() const
{
	return m_Connected;
}

int BoardIO::revision() const
{
	return m_Revision;
}

int BoardIO::gpioChip() const
{
	return m_GpioChip;
}

void BoardIO::initializePin(int gpio, int mode, int value)
{
	if (!isConnected())
		return;
	pinMode(gpio, mode);
	if (mode == OUTPUT)
		digitalWrite(gpio, value);
	else
		pullUpDnControl(gpio, (value == 0) ? PUD_DOWN : PUD_UP);
}

bool BoardIO::setOut1(int value)
{
	write(OUT1_PIN, value);
	return (value == read(OUT1_PIN));
}

bool BoardIO::setOut2(int value)
{
	write(OUT2_PIN, value);
	return (value == read(OUT2_PIN));
}

void BoardIO::write(int gpio, int value)
{
	if (!isConnected())
		return;
	digitalWrite(gpio, value);
}

int BoardIO::read(int gpio) const
{
	if (!isConnected())
		return -1;
	return digitalRead(gpio);
}

int BoardIO::detectBoard()
{
	const std::string model = readFile("/proc/device-tree/model");

	if (model.find("Raspberry Pi 5") != std::string::npos)
		return RP5_GPIOCHIP;

	if (model.find("Raspberry Pi 4") != std::string::npos)
		return RP4_GPIOCHIP;

	return RP_UNKNOWN;
}

int BoardIO::checkRevision()
{
	int rev = 1;

	initializePin(MOTOR_PWM, INPUT, LOW);  // pin38
	initializePin(CHK_IN_PIN, INPUT, LOW); // pin36

	DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "Set DAC");
	setDacHold(0);
	std::this_thread::sleep_for(std::chrono::milliseconds(5));
	if (read(MOTOR_PWM) == 0)
	{
		setDacHold(255);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		if (read(MOTOR_PWM) == 1)
			rev = 2;
	}

	setDacHold(0);
	std::this_thread::sleep_for(std::chrono::milliseconds(5));
	if (read(CHK_IN_PIN) == 0)
	{
		setDacHold(255);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		if (read(CHK_IN_PIN) == 1)
			rev = 3;
	}

	initializePin(MOTOR_PWM, OUTPUT, LOW);
	if (rev == 1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		if (read(CHK_IN_PIN) == 0)
		{
			write(MOTOR_PWM, 1);
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			if (read(CHK_IN_PIN) == 1)
			{
				rev = 4;
			}
		}
	}
	initializePin(MOTOR_PWM, INPUT, LOW);

	DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "AstroLink 4 Pi revision %d detected", rev);
	return rev;
}

std::string BoardIO::readFile(const std::string &path) const
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return {};

	std::ostringstream ss;
	ss << in.rdbuf();
	return ss.str();
}

int BoardIO::setDacRun(int value)
{
	return setDac(m_Config.dacChannelRun, value);
}

int BoardIO::setDacHold(int value)
{
	return setDac(m_Config.dacChannelHold, value);
	// return writeDac(m_Config.dacChannelHold, value);
}

int BoardIO::setDac(int channel, int value)
{
	int setupResult = wiringPiSPIxSetupMode(0, m_Config.spiChannel, m_Config.spiSpeed, 0);
	if (setupResult < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
					 "wiringPiSPIxSetupMode failed: errno=%d (%s)", errno, std::strerror(errno));
		return -1;
	}
	DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "Open %d %d", setupResult, wiringPiSPIxGetFd(0, m_Config.spiChannel));

	struct SpiCloser
	{
		int controller;
		int channel;
		~SpiCloser()
		{
			int close = wiringPiSPIxClose(controller, channel);
			DEBUGFDEVICE("SPI closer", INDI::Logger::DBG_WARNING, "Closer %d", close);
		}
	};

	SpiCloser spiCloser{0, m_Config.spiChannel};

	channel = (channel != 0) ? 1 : 0;
	value = clampInt(value, DAC_MIN_VALUE, DAC_MAX_VALUE);

	DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "Setting DAC chan %d val %d", channel, value);

	uint16_t data = 0;

	// Frame:
	// bit15: 0
	// bit14: channel (0=A, 1=B)
	// bit13: 1 (aktywny DAC)
	// bit12: 1 (gain = 1x)
	// bit11-4: dane (8-bit)
	// bit3-0: don't care

	data |= (channel & 0x01) << 15; // wybór kanału
	data |= (1 << 13);				// aktywacja
	data |= (1 << 12);				// gain = 1x
	data |= (value << 4);			// dane

	uint8_t buffer[2];
	buffer[0] = (data >> 8) & 0xFF;
	buffer[1] = data & 0xFF;

	int returnValue = wiringPiSPIxDataRW(0, m_Config.spiChannel, buffer, 2);
	if (returnValue < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
					 "wiringPiSPIxDataRW failed: errno=%d (%s)", errno, std::strerror(errno));
		return -1;
	}
	return returnValue;
}

// channel: 0 = DAC A, 1 = DAC B
// value:   0..255 dla MCP4802 (8-bit)
// gain1x:  true = 1x, false = 2x
// active:  true = aktywny DAC, false = shutdown
bool BoardIO::writeDac(uint8_t channel,
					   uint8_t value,
					   bool gain1x,
					   bool active,
					   const char *device,
					   uint32_t speedHz)
{
	channel = (channel != 0) ? 1 : 0;
	value = clampInt(value, DAC_MIN_VALUE, DAC_MAX_VALUE);

	DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "Opening device");
	int fd = ::open(device, O_RDWR);
	DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "Opening device %d", fd);
	if (fd < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
					 "writeDac open failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}
	else
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING, "writeDac open OK: fd=%d", fd);
	}

	// Gwarancja zamknięcia przy KAŻDYM wyjściu z funkcji
	struct FdGuard
	{
		int fd_;
		~FdGuard()
		{
			if (fd_ >= 0)
				::close(fd_);
		}
	} guard{fd};

	uint8_t mode = SPI_MODE_0;
	uint8_t bitsPerWord = 8;

	if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
					 "writeDac SPI_IOC_WR_MODE failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	if (::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
					 "writeDac SPI_IOC_WR_BITS_PER_WORD failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	if (::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speedHz) < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
					 "writeDac SPI_IOC_WR_MAX_SPEED_HZ failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	// MCP4802, ramka 16-bit:
	// bit15    : 0
	// bit14    : channel (0=A, 1=B)
	// bit13    : active  (1=active, 0=shutdown)
	// bit12    : gain    (1=1x, 0=2x)
	// bit11    : 0
	// bit10..3 : data D7..D0
	// bit2..0  : 0
	uint16_t frame = 0;
	frame |= (static_cast<uint16_t>(channel ? 1 : 0) << 15);
	frame |= (static_cast<uint16_t>(gain1x ? 1 : 0) << 13);
	frame |= (static_cast<uint16_t>(active ? 1 : 0) << 12);
	frame |= (static_cast<uint16_t>(value) << 4);

	uint8_t tx[2];
	tx[0] = static_cast<uint8_t>((frame >> 8) & 0xFF);
	tx[1] = static_cast<uint8_t>(frame & 0xFF);

	struct spi_ioc_transfer tr;
	std::memset(&tr, 0, sizeof(tr));
	tr.tx_buf = reinterpret_cast<unsigned long>(tx);
	tr.rx_buf = 0;
	tr.len = sizeof(tx);
	tr.speed_hz = speedHz;
	tr.bits_per_word = bitsPerWord;

	// Jeden transfer 2-bajtowy
	if (::ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
					 "writeDac SPI_IOC_MESSAGE failed: errno=%d (%s)", errno, std::strerror(errno));
		return false;
	}

	return true;
}
