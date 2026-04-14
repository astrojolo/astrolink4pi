
/*******************************************************************************
 Copyright(c) 2026 astrojolo.com
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

#include <atomic>
#include <cstdio>
#include <mutex>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <memory>
#include <cmath>
#include <ctime>
#include <thread>
#include <algorithm>
#include <cstdint>
#include <vector>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include "config.h"

#include "basecomponent.h"
#include "boardio.h"
#include "pwm.h"
#include "systeminfo.h"
#include "powermonitor.h"
#include "shtreader.h"
#include "mlxreader.h"
#include "tslreader.h"
#include "dsreader.h"
#include "focuser.h"

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
	virtual int savePosition(int pos);
	virtual bool readSHT();
	virtual bool readMLX();
	virtual bool readSQM(bool triggerOldSensor);
	virtual bool readTSL();
	virtual bool readOLD();
	virtual bool readPower();
	virtual bool readDS18B20();

	BoardIO m_BoardIO;
	PwmController m_PwmController;
	SystemInfoService m_SystemInfo;
	PowerMonitor m_PowerMonitor;
	SHTReader m_SHTReader;
	// MLXReader m_MLXReader;
	// TSLReader m_TSLReader;
	// DSFileReader m_DSReader;
	// Focuser::Config m_FocuserConfig;
	// Focuser m_Focuser;

	ISwitch FocusResolutionS[6];
	ISwitchVectorProperty FocusResolutionSP;
	enum
	{
		RES_1,
		RES_2,
		RES_4,
		RES_8,
		RES_16,
		RES_32
	};
	ISwitch FocusHoldS[6];
	ISwitchVectorProperty FocusHoldSP;
	enum
	{
		HOLD_0,
		HOLD_20,
		HOLD_40,
		HOLD_60,
		HOLD_80,
		HOLD_100
	};
	INumber FocusStepDelayN[1];
	INumberVectorProperty FocusStepDelayNP;
	INumber FocusTemperatureN[1];
	INumberVectorProperty FocusTemperatureNP;
	INumber TemperatureCoefN[1];
	INumberVectorProperty TemperatureCoefNP;
	ISwitch TemperatureCompensateS[2];
	ISwitchVectorProperty TemperatureCompensateSP;

	INumber SQMOffsetN[1];
	INumberVectorProperty SQMOffsetNP;
	enum class TSLState
	{
		NotAvailable,
		Available,
		Initialized
	};

	INumber FocuserInfoN[3];
	INumberVectorProperty FocuserInfoNP;
	enum
	{
		FOC_STEP_SIZE,
		FOC_CFZ,
		FOC_STEPS_CFZ
	};
	INumber FocuserTravelN[1];
	INumberVectorProperty FocuserTravelNP;

	INumber FanPowerN[1];
	INumberVectorProperty FanPowerNP;

	INumber PowerReadingsN[6];
	INumberVectorProperty PowerReadingsNP;
	enum
	{
		POW_VIN,
		POW_VREG,
		POW_PTOT,
		POW_ITOT,
		POW_AH,
		POW_WH
	};

	INumber ScopeParametersN[2];
	INumberVectorProperty ScopeParametersNP;
	enum
	{
		SCOPE_DIAM,
		SCOPE_FL
	};

	IText SysTimeT[2];
	ITextVectorProperty SysTimeTP;
	enum
	{
		SYST_TIME,
		SYST_OFFSET
	};
	IText SysInfoT[7];
	ITextVectorProperty SysInfoTP;
	enum
	{
		SYSI_HARDWARE,
		SYSI_CPUTEMP,
		SYSI_UPTIME,
		SYSI_LOAD,
		SYSI_HOST,
		SYSI_LOCALIP,
		SYSI_PUBIP
	};

	IText RelayLabelsT[4];
	ITextVectorProperty RelayLabelsTP;
	enum
	{
		LAB_OUT1,
		LAB_OUT2,
		LAB_PWM1,
		LAB_PWM2
	};

	ISwitch Switch1S[2];
	ISwitchVectorProperty Switch1SP;
	enum
	{
		S1_ON,
		S1_OFF
	};
	ISwitch Switch2S[2];
	ISwitchVectorProperty Switch2SP;
	enum
	{
		S2_ON,
		S2_OFF
	};

	INumber PWM1N[1];
	INumberVectorProperty PWM1NP;
	INumber PWM2N[1];
	INumberVectorProperty PWM2NP;

	INumber StepperCurrentN[1];
	INumberVectorProperty StepperCurrentNP;

	enum class SensorCycle
	{
		SHT = 1,
		MLX,
		TSL,
		SQM,
		IDLE
	};

	SensorCycle m_Cycle = SensorCycle::IDLE;

	inline SensorCycle &operator++(SensorCycle &c)
	{
		if (c == SensorCycle::IDLE)
			c = SensorCycle::SHT;
		else
			c = static_cast<SensorCycle>(static_cast<int>(c) + 1);

		return c;
	}

	int getHoldPower();
	void getFocuserInfo();
	void temperatureCompensation();
	void systemUpdate();
	void fanUpdate();

	static constexpr const char *ENVIRONMENT_TAB{"Environment"};
	static constexpr const char *SYSTEM_TAB{"System"};
	static constexpr const char *OUTPUTS_TAB{"Outputs"};
};

#endif
