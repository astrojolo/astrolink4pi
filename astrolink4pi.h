
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

#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <math.h>
#include <memory>
#include <time.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include "config.h"

#include <lgpio.h>

#include <defaultdevice.h>
#include <indifocuserinterface.h>
#include <indiweatherinterface.h>

class AstroLink4Pi : public INDI::DefaultDevice, public INDI::FocuserInterface, public INDI::WeatherInterface
{
public:
	AstroLink4Pi();
	virtual ~AstroLink4Pi();

	virtual bool initProperties();
	virtual bool updateProperties();

	virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
	virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
	virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n);

protected:
	const char *getDefaultName();

	// Focuser Overrides
	virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
	virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
	virtual bool ReverseFocuser(bool enabled);
	virtual bool AbortFocuser();
	virtual bool SyncFocuser(uint32_t ticks) override;
	virtual bool SetFocuserBacklash(int32_t steps) override;
	virtual bool SetFocuserMaxPosition(uint32_t ticks) override;

	virtual bool saveConfigItems(FILE *fp);
	virtual void TimerHit();

	// Weather Overrides
    virtual IPState updateWeather() override
    {
        return IPS_OK;
    }

private:
	virtual bool Connect();
	virtual bool Disconnect();
	virtual void SetResolution(int res);
	virtual int savePosition(int pos);
	virtual bool readSHT();
	virtual bool readMLX();
	virtual bool readSQM();
	virtual bool readPower();

	ISwitch FocusResolutionS[6];
	ISwitchVectorProperty FocusResolutionSP;
	enum {RES_1, RES_2, RES_4, RES_8, RES_16, RES_32};
	ISwitch FocusHoldS[6];
	ISwitchVectorProperty FocusHoldSP;
	enum {HOLD_0, HOLD_20, HOLD_40, HOLD_60, HOLD_80, HOLD_100};
	INumber FocusStepDelayN[1];
	INumberVectorProperty FocusStepDelayNP;
	INumber FocusTemperatureN[1];
	INumberVectorProperty FocusTemperatureNP;
	INumber TemperatureCoefN[1];
	INumberVectorProperty TemperatureCoefNP;
	ISwitch TemperatureCompensateS[2];
	ISwitchVectorProperty TemperatureCompensateSP;

	INumber FocuserInfoN[3];
	INumberVectorProperty FocuserInfoNP;
	enum {FOC_STEP_SIZE, FOC_CFZ, FOC_STEPS_CFZ};
	INumber FocuserTravelN[1];
	INumberVectorProperty FocuserTravelNP;

	INumber FanPowerN[1];
	INumberVectorProperty FanPowerNP;

	INumber PowerReadingsN[6];
	INumberVectorProperty PowerReadingsNP;
    enum { POW_VIN, POW_VREG, POW_PTOT, POW_ITOT, POW_AH, POW_WH };	

	INumber ScopeParametersN[2];
	INumberVectorProperty ScopeParametersNP;
	enum {SCOPE_DIAM, SCOPE_FL};

	IText SysTimeT[2];
	ITextVectorProperty SysTimeTP;
	IText SysInfoT[7];
	ITextVectorProperty SysInfoTP;

	IText RelayLabelsT[4];
	ITextVectorProperty RelayLabelsTP;
	enum {LAB_OUT1, LAB_OUT2, LAB_PWM1, LAB_PWM2};

	ISwitch Switch1S[2];
	ISwitchVectorProperty Switch1SP;
	enum {S1_ON, S1_OFF};
	ISwitch Switch2S[2];
	ISwitchVectorProperty Switch2SP;
	enum {S2_ON, S2_OFF};

	INumber PWM1N[1];
	INumberVectorProperty PWM1NP;
	INumber PWM2N[1];
	INumberVectorProperty PWM2NP;

	INumber PWMcycleN[1];
	INumberVectorProperty PWMcycleNP;

	INumber StepperCurrentN[1];
	INumberVectorProperty StepperCurrentNP;

	int revision = 1;
	int gpioType = 0;
	int gpioChip = -1;
	int pigpioHandle = -1;

	int resolution = 1;

	float lastTemperature;
	float focuserTemperature;
	bool SHTavailable = false;
	bool MLXavailable = false;
	bool SQMavailable = false;

	int backlashTicksRemaining;
	int lastDirection = 0;

	int pwmState[2];
	int relayState[2];

	long int nextTemperatureRead = 0;
	long int nextTemperatureCompensation = 0;
	long int nextSystemRead = 0;
	long int nextFanUpdate = 0;

	int powerIndex = 0;
	float energyAs = 0.0;
	float energyWs = 0.0;

	std::thread _motionThread;
	volatile bool _abort;

	int getHoldPower();
	void getFocuserInfo();
	void temperatureCompensation();
	void setCurrent(bool standby);
	void systemUpdate();
	void fanUpdate();
	int getMotorPWM(int current);
	int checkRevision();
	long int millis();
	std::thread getMotorThread(uint32_t targetPos, int direction, int pigpioHandle, int backlashTicksRemaining);

    static constexpr const char *ENVIRONMENT_TAB {"Environment"};
	static constexpr const char *SYSTEM_TAB{"System"};
	static constexpr const char *OUTPUTS_TAB{"Outputs"};
};

#endif
