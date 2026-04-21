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

	int pin = 20; // BCM20

	initializePin(20, INPUT, LOW);  // pin38

    while (true)
    {
        // HIGH
        write(pin, HIGH);
        delay(3000); // 3 sekundy

        // LOW
        write(pin, LOW);
        delay(3000); // 3 sekundy
    }

    return false;	

	initializePin(OUT1_PIN, OUTPUT, HIGH);
	initializePin(OUT2_PIN, OUTPUT, LOW);


	m_SpiFd = wiringPiSPISetup(m_Config.spiChannel, m_Config.spiSpeed);
	if (m_SpiFd < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_DEBUG,
					 "wiringPiSPISetup failed: errno=%d (%s)", errno, std::strerror(errno));
	}
	else
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "wiringPiSPISetup ok SpiFid %d", m_SpiFd);
	}

	m_GpioChip = detectBoard();
	m_Revision = checkRevision();

	return true;
}

void BoardIO::disconnect()
{
	m_SpiFd = -1;
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
	if (read(MOTOR_PWM) == 0)
	{
		setDacHold(255);
		if (read(MOTOR_PWM) == 1)
			rev = 2;
	}

	setDacHold(0);
	if (read(CHK_IN_PIN) == 0)
	{
		setDacHold(255);
		if (read(CHK_IN_PIN) == 1)
			rev = 3;
	}

	initializePin(MOTOR_PWM, OUTPUT, LOW);
	if (rev == 1)
	{
		if (read(CHK_IN_PIN) == 0)
		{
			write(MOTOR_PWM, 1);
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

int BoardIO::setDac(int chan, int value)
{
	unsigned char spiData[2];
	unsigned char chanBits, dataBits;

	if (chan == 0)
		chanBits = 0x30;
	else
		chanBits = 0xB0;

	chanBits |= ((value >> 4) & 0x0F);
	dataBits = ((value << 4) & 0xF0);

	spiData[0] = chanBits;
	spiData[1] = dataBits;

	return wiringPiSPIDataRW(m_Config.spiChannel, spiData, 2);
}

// int BoardIO::setDac(int chan, int value)
// {
// 	if (m_SpiFd < 0)
// 	{
// 		DEBUGDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "SPI not available - write error.");
// 		return -1;
// 	}

// 	chan = (chan != 0) ? 1 : 0;
// 	value = clampInt(value, DAC_MIN_VALUE, DAC_MAX_VALUE);

// 	DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_SESSION, "Setting DAC chan %d val %d", chan, value);

// 	// MCP4802 16-bit frame:
// 	// bit15 A/B
// 	// bit14 don't care (0)
// 	// bit13 GA = 1 (1x)
// 	// bit12 SHDN = 1 (active)
// 	// bits11..4 = 8-bit DAC data
// 	// bits3..0  = don't care (0)
// 	uint16_t frame = 0;
// 	frame |= (static_cast<uint16_t>(chan) << 15);
// 	frame |= (1u << 13);
// 	frame |= (1u << 12);
// 	frame |= (static_cast<uint16_t>(value & 0xFF) << 4);

// 	unsigned char data[2];
// 	data[0] = static_cast<unsigned char>((frame >> 8) & 0xFF);
// 	data[1] = static_cast<unsigned char>(frame & 0xFF);

// 	return wiringPiSPIDataRW(m_SpiFd, data, 2);
// }