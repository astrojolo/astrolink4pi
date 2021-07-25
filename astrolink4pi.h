/*******************************************************************************
 Copyright(c) 2021 astrojolo.com
 .
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#ifndef ASTROLINK4PI_H
#define ASTROLINK4PI_H

#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <memory>
#include <regex>
#include <cstring>
#include <map>
#include <sstream>

#include <defaultdevice.h>
#include <indifocuserinterface.h>


class IndiAstrolink4Pi : public INDI::DefaultDevice, public INDI::FocuserInterface
{

public:
    IndiAstrolink4Pi();
	virtual ~IndiAstrolink4Pi();
	virtual const char *getDefaultName();
	virtual bool initProperties();
	virtual bool updateProperties();
	virtual void ISGetProperties(const char *dev);
	virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
	virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
	virtual bool ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);
	virtual bool ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n);
	virtual bool ISSnoopDevice(XMLEle *root);
	static void stepperStandbyHelper(void *context);
	static void updateTemperatureHelper(void *context);
	static void temperatureCompensationHelper(void *context);
	static void stepperRunHelper(void *context);

protected:
	virtual bool saveConfigItems(FILE *fp);
	virtual void TimerHit();

    // Focuser Overrides
    virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    virtual bool AbortFocuser() override;
    virtual bool ReverseFocuser(bool enabled) override;
    virtual bool SyncFocuser(uint32_t ticks) override;

private:
	virtual bool Connect();
	virtual bool Disconnect();
	virtual void SetResolution(int res);
	virtual int savePosition(int pos);
	virtual bool readDS18B20();
	virtual void stepMotor(int direction);
	virtual void stepperRun();	

	IText SysTimeT[2];
	ITextVectorProperty SysTimeTP;
	IText SysInfoT[8];
	ITextVectorProperty SysInfoTP;
	ISwitch SysControlS[2];
	ISwitchVectorProperty SysControlSP;
	INumber FanTempN[1];
	INumberVectorProperty FanTempNP;
	INumber BCMpinsN[1];
	INumberVectorProperty BCMpinsNP;	
	ISwitch SysOpConfirmS[2];
	ISwitchVectorProperty SysOpConfirmSP;

	ISwitch MotorDirS[2];
	ISwitchVectorProperty MotorDirSP;
	ISwitch FocusResolutionS[2];
	ISwitchVectorProperty FocusResolutionSP;
	INumber FocuserInfoN[3];
	INumberVectorProperty FocuserInfoNP;
	INumber FocusStepDelayN[1];
	INumberVectorProperty FocusStepDelayNP;
	INumber FocusBacklashN[1];
	INumberVectorProperty FocusBacklashNP;
	INumber FocuserTravelN[1];
	INumberVectorProperty FocuserTravelNP;
	ISwitch ResetAbsPosS[1];
	ISwitchVectorProperty ResetAbsPosSP;
	IText ActiveTelescopeT[1];
	ITextVectorProperty ActiveTelescopeTP;
	INumber ScopeParametersN[2];
	INumberVectorProperty ScopeParametersNP;
	INumber FocusTemperatureN[1];
	INumberVectorProperty FocusTemperatureNP;
	INumber TemperatureCoefN[1];
	INumberVectorProperty TemperatureCoefNP;
	ISwitch TemperatureCompensateS[2];
	ISwitchVectorProperty TemperatureCompensateSP;

	const char* gpio_chip_path = "/dev/gpiochip0";
	struct gpiod_chip *chip;
	struct gpiod_line *gpio_sysfan;
	struct gpiod_line *gpio_a1;
	struct gpiod_line *gpio_a2;
	struct gpiod_line *gpio_b1;
	struct gpiod_line *gpio_b2;

	int resolution = 1;
	float lastTemperature;

	int backlashTicksRemaining;
	int ticksRemaining;
	int lastDirection = 0;
	int currentStep = -1;
	bool abortStep = false;

	void stepperRun();
	int stepperRunID { -1 };
	void getFocuserInfo();
	int stepperStandbyID { -1 };
	void stepperStandby();
	int updateTemperatureID { -1 };
	void updateTemperature();
	int temperatureCompensationID { -1 };
	void temperatureCompensation();

};

#endif