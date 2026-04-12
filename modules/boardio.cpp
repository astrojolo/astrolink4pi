#include "boardio.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

BoardIO::BoardIO()
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

    return true;
}

void BoardIO::disconnect()
{
    // nothig to do
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


void BoardIO::initializePin(int gpio, int mode, int value)
{
    if(!isConnected()) return;
	pinMode(gpio, mode);
	if (mode == OUTPUT)
		digitalWrite(gpio, value);
	else
		pullUpDnControl(gpio, (value == 0) ? PUD_DOWN : PUD_UP);
}

void BoardIO::write(int gpio, int value)
{
    if(!isConnected()) return;
    digitalWrite(gpio, value);
}

int BoardIO::read(int gpio) const
{
    if(!isConnected()) return -1;
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
	int rev = 4;
	// int handle = lgGpiochipOpen(RP5_GPIO);

	// if (handle < 0)
	// {
	// 	handle = lgGpiochipOpen(RP4_GPIO);
	// 	if (handle < 0)
	// 		DEBUG(INDI::Logger::DBG_SESSION, "Neither RPi4 nor RPi5 GPIO was detected.\n");
	// 	else
	// 		gpioType = RP4_GPIO;
	// }
	// else
	// {
	// 	gpioType = RP5_GPIO;
	// }

	// lgChipInfo_t cInfo;
	// int status = lgGpioGetChipInfo(handle, &cInfo);

	// if (status == LG_OKAY)
	// {
	// 	DEBUGF(INDI::Logger::DBG_SESSION, "GPIO chip lines=%d name=%s label=%s\n", cInfo.lines, cInfo.name, cInfo.label);
	// 	lgpioHandle = handle;
	// }

	// int spiHandle = lgSpiOpen(lgpioHandle, 1, 100000, 0);
	// if (spiHandle >= 0)
	// {
	// 	DEBUG(INDI::Logger::DBG_SESSION, "SPI bus active.\n");
	// 	lgSpiClose(spiHandle);
	// }
	// int i2cHandle = lgI2cOpen(1, 0x68, 0);
	// if (i2cHandle >= 0)
	// {
	// 	DEBUG(INDI::Logger::DBG_SESSION, "I2C bus active.\n");
	// 	lgI2cClose(i2cHandle);
	// }

	// lgGpioClaimInput(handle, 0, MOTOR_PWM);	 // OLD CHK_PIN
	// lgGpioClaimInput(handle, 0, CHK_IN_PIN); // OLD CHK2_PIN

	// setDac(1, 0);
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