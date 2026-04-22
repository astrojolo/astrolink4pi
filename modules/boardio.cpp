#include "boardio.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp4802.h>
#include <stdio.h>

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

	initializePin(OUT1_PIN, OUTPUT, HIGH);
	initializePin(OUT2_PIN, OUTPUT, LOW);

	m_GpioChip = detectBoard();
	m_Revision = checkRevision();

	return true;
}

void BoardIO::disconnect()
{
    if (m_SpiFd >= 0)
    {
        setDacRun(0);
        setDacHold(0);
        wiringPiSPIClose(m_Config.spiChannel);
        m_SpiFd = -1;
    }
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

	DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "AstroLink 4 Pi revision %d detected", rev);
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
}

int BoardIO::setDac(int channel, int value)
{
	int setupResult = wiringPiSPIxSetupMode(0, m_Config.spiChannel, m_Config.spiSpeed, 0);
	if (setupResult < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
					 "wiringPiSPIxSetupMode failed: errno=%d (%s)", errno, std::strerror(errno));
		return -1;
	}

	struct SpiCloser
	{
		int controller;
		int channel;
		~SpiCloser()
		{
			wiringPiSPIxClose(controller, channel);
		}
	};

	SpiCloser spiCloser{0, m_Config.spiChannel};	

	channel = (channel != 0) ? 1 : 0;
	value = clampInt(value, DAC_MIN_VALUE, DAC_MAX_VALUE);

	DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG, "Setting DAC chan %d val %d", channel, value);

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
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
					 "wiringPiSPIxDataRW failed: errno=%d (%s)", errno, std::strerror(errno));
		return -1;
	}	
	return returnValue;
}