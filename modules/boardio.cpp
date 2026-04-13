#include "boardio.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace
{
    constexpr int DAC_MAX_VALUE = 4095;
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

	m_GpioChip = detectBoard();
	m_Revision = checkRevision();

	int wiringPiSetup = wiringPiSetupPinType(WPI_PIN_BCM);
	if (wiringPiSetup < 0)
	{
		return false;
	}

	m_SpiFd = wiringPiSPISetup(m_Config.spiChannel, m_Config.spiSpeed);
	if (m_SpiFd < 0)
	{
		DEBUGFDEVICE(getDeviceName().c_str(), INDI::Logger::DBG_WARNING,
					 "wiringPiSPISetup failed: errno=%d (%s)", errno, std::strerror(errno));
	}

	return true;
}

void BoardIO::disconnect()
{
	m_SpiFd = -1;
}

bool BoardIO::isConnected() const
{
	return m_Revision > 0;
}

int BoardIO::revision() const
{
	return m_Revision;
}

int BoardIO::gpioChip() const
{
	return m_GpioChip;
}

BoardIO::Config BoardIO::getConfig() const
{
	return m_Config;
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

	return RP4_GPIOCHIP;
}

int BoardIO::checkRevision()
{
	// TODO - check SPI, I2C and 1-Wire

	int rev = 1;
	initializePin(m_Config.pinCHK_IN, INPUT, LOW);
	initializePin(m_Config.pinCHK2_IN, INPUT, LOW);

	// lgGpioClaimInput(handle, 0, MOTOR_PWM);	 // OLD CHK_PIN
	// lgGpioClaimInput(handle, 0, MOTOR_PWM); // OLD CHK2_PIN

	setDac(1, 0);
	// if (lgGpioRead(handle, MOTOR_PWM) == 0)
	// {
	// 	setDac(1, 255);
	// 	if (lgGpioRead(handle, MOTOR_PWM) == 1)
	// 		rev = 2;
	// }

	// setDac(1, 0);
	// if (lgGpioRead(handle, CHK_IN_PIN) == 0)
	// {
	// 	setDac(1, 255);
	// 	if (lgGpioRead(handle, CHK_IN_PIN) == 1)
	// 		rev = 3;
	// }

	// lgGpioClaimOutput(handle, 0, MOTOR_PWM, 0);
	// if (rev == 1)
	// {
	// 	if (lgGpioRead(handle, CHK_IN_PIN) == 0)
	// 	{
	// 		lgGpioWrite(handle, MOTOR_PWM, 1);		 // pin20
	// 		if (lgGpioRead(handle, CHK_IN_PIN) == 1) // pin16
	// 		{
	// 			rev = 4;
	// 		}
	// 	}
	// }
	// lgGpioFree(handle, MOTOR_PWM);
	// lgGpioFree(handle, CHK_IN_PIN);

	// if (handle >= 0)
	// 	lgGpiochipClose(handle);

	// DEBUGF(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi revision %d detected", rev);
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

int BoardIO::setDac(int chan, int value)
{
	if (m_SpiFd < 0)
		return -1;

	chan = (chan != 0) ? 1 : 0;
	//value = clampInt(value, DAC_MIN_VALUE, DAC_MAX_VALUE);

	// MCP4922-style 16-bit frame:
	// bit15 A/B, bit14 BUF=0, bit13 GA=1 (1x), bit12 SHDN=1, bits11..0 data
	uint16_t frame = 0;
	frame |= (static_cast<uint16_t>(chan) << 15);
	frame |= (1u << 13);
	frame |= (1u << 12);
	frame |= static_cast<uint16_t>(value & 0x0FFF);

	unsigned char data[2];
	data[0] = static_cast<unsigned char>((frame >> 8) & 0xFF);
	data[1] = static_cast<unsigned char>(frame & 0xFF);

	return wiringPiSPIDataRW(m_Config.spiChannel, data, 2);
}