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
#include <stdio.h>
#include <memory>
#include <numeric>
#include <string.h>
#include "config.h"

#include <gpiod.h>

#include "astrolink4pi.h"

//////////////////////////////////////////////////////////////////////
/// Delegates
//////////////////////////////////////////////////////////////////////
std::unique_ptr<IndiAstrolink4Pi> indiIndiAstrolink4Pi(new IndiAstrolink4Pi());

#define TIMER_POLL                          1000        // ms
#define MAX_RESOLUTION                      2           // the highest resolution supported is 1/2 step
#define TEMPERATURE_UPDATE_TIMEOUT          (3 * 1000)  // 3 sec
#define STEPPER_STANDBY_TIMEOUT             (2 * 1000)  // 2 sec
#define TEMPERATURE_COMPENSATION_TIMEOUT    (30 * 1000) // 30 sec
#define STEPPER_CYCLE                       5           // ms

#define PIN_SYS_FAN		9
#define PIN_STP_A1      10
#define PIN_STP_A2      11
#define PIN_STP_B1      12
#define PIN_STP_B2      13

int halfStep[8][4] = { {1,0,0,0}, {1,0,1,0}, {0,0,1,0}, {0,1,1,0}, {0,1,0,0}, {0,1,0,1}, {0,0,0,1}, {1,0,0,1} };
int fullStep[8][4] = { {1,0,0,0}, {0,0,1,0}, {0,1,0,0}, {0,0,0,1}, {1,0,0,0}, {0,0,1,0}, {0,1,0,0}, {0,0,0,1} };

void ISPoll(void *p);

void ISInit()
{
	static int isInit = 0;

	if (isInit == 1)
		return;
	if(indiIndiAstrolink4Pi.get() == 0)
	{
		isInit = 1;
		indiIndiAstrolink4Pi.reset(new IndiAstrolink4Pi());
	}
}

void ISGetProperties(const char *dev)
{
        ISInit();
        indiIndiAstrolink4Pi->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
        ISInit();
        indiIndiAstrolink4Pi->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
        ISInit();
        indiIndiAstrolink4Pi->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
        ISInit();
        indiIndiAstrolink4Pi->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int num)
{
	INDI_UNUSED(dev);
	INDI_UNUSED(name);
	INDI_UNUSED(sizes);
	INDI_UNUSED(blobsizes);
	INDI_UNUSED(blobs);
	INDI_UNUSED(formats);
	INDI_UNUSED(names);
	INDI_UNUSED(num);
}


//////////////////////////////////////////////////////////////////////
///Constructor
//////////////////////////////////////////////////////////////////////

IndiAstrolink4Pi::IndiAstrolink4Pi() : FI(this)
{
	setVersion(VERSION_MAJOR,VERSION_MINOR);
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_REVERSE | FOCUSER_CAN_SYNC | FOCUSER_CAN_ABORT); 
	Focuser::setSupportedConnections(CONNECTION_NONE);
}

IndiAstrolink4Pi::~IndiAstrolink4Pi()
{
	// Delete controls on options tab
}

const char * IndiAstrolink4Pi::getDefaultName()
{
    return (char *)"AstroLink 4 Pi";
}

//////////////////////////////////////////////////////////////////////
/// Communication
//////////////////////////////////////////////////////////////////////
void IndiAstrolink4Pi::TimerHit()
{
    if(isConnected())
    {
		// update time
		struct tm *local_timeinfo;
		static char ts[32];
		time_t rawtime;
		time(&rawtime);
		local_timeinfo = localtime (&rawtime);
		strftime(ts, 20, "%Y-%m-%dT%H:%M:%S", local_timeinfo);
		IUSaveText(&SysTimeT[0], ts);
		snprintf(ts, sizeof(ts), "%4.2f", (local_timeinfo->tm_gmtoff/3600.0));
		IUSaveText(&SysTimeT[1], ts);
		SysTimeTP.s = IPS_OK;
		IDSetText(&SysTimeTP, NULL);

		SysInfoTP.s = IPS_BUSY;
		IDSetText(&SysInfoTP, NULL);

		FILE* pipe;
		char buffer[128];

		//update CPU temp
		pipe = popen("echo $(($(cat /sys/class/thermal/thermal_zone0/temp)/1000))", "r");
		if(fgets(buffer, 128, pipe) != NULL) IUSaveText(&SysInfoT[1], buffer);
		pclose(pipe);
		
		//update uptime
		pipe = popen("uptime|awk -F, '{print $1}'|awk -Fup '{print $2}'|xargs", "r");
		if(fgets(buffer, 128, pipe) != NULL) IUSaveText(&SysInfoT[2], buffer);
		pclose(pipe);
		
		//update load
		pipe = popen("uptime|awk -F, '{print $3\" /\"$4\" /\"$5}'|awk -F: '{print $2}'|xargs", "r");
		if(fgets(buffer, 128, pipe) != NULL) IUSaveText(&SysInfoT[3], buffer);
		pclose(pipe);
		
		SysInfoTP.s = IPS_OK;
		IDSetText(&SysInfoTP, NULL);        
        SetTimer(TIMER_POLL);
    }
}

bool IndiAstrolink4Pi::Connect()
{
	// Init GPIO
	chip = gpiod_chip_open(gpio_chip_path);
	if (!chip)
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Problem initiating AstroLink 4 Pi GPIO chip.");
		return false;
	}

	// verify BCM fan pin is not used by other consumers
    int pins[] = {PIN_SYS_FAN, PIN_STP_A1, PIN_STP_A2, PIN_STP_B1, PIN_STP_B2};
	for (unsigned int pin = 0; pin < 5; pin++)
	{
		if (gpiod_line_is_used(gpiod_chip_get_line(chip, pins[pin])))
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "BCM Pin %0.0f already used", pins[pin]);
			gpiod_chip_close(chip);
			return false;
		}
	}

	// Select gpios
	gpio_sysfan = gpiod_chip_get_line(chip, PIN_SYS_FAN);
    gpio_a1 = gpiod_chip_get_line(chip, PIN_STP_A1);
	gpio_a2 = gpiod_chip_get_line(chip, PIN_STP_A2);
	gpio_b1 = gpiod_chip_get_line(chip, PIN_STP_B1);
	gpio_b2 = gpiod_chip_get_line(chip, PIN_STP_B2);
	// Set initial gpios direction and states
	gpiod_line_request_output(gpio_sysfan, "astroberry_sysfan", 0);
    gpiod_line_request_output(gpio_a1, "a1@astroberry_focuser", 0);
	gpiod_line_request_output(gpio_a2, "a2@astroberry_focuser", 0);
	gpiod_line_request_output(gpio_b1, "b1@astroberry_focuser", 0); 
	gpiod_line_request_output(gpio_b2, "b2@astroberry_focuser", 0);

	//read last position from file & convert from MAX_RESOLUTION to current resolution
	FocusAbsPosN[0].value = savePosition(-1) != -1 ? (int) savePosition(-1) * resolution / MAX_RESOLUTION : 0;

	// preset resolution
	SetResolution(resolution);

	// Update focuser parameters
	getFocuserInfo();
    
	// set motor standby timer
	stepperStandbyID = IEAddTimer(STEPPER_STANDBY_TIMEOUT, stepperStandbyHelper, this);    

	SetTimer(TIMER_POLL);
	IDMessage(getDeviceName(), "AstroLink 4 Pi System connected successfully.");

	// Get basic system info
	FILE* pipe;
	char buffer[128];

	//update Hardware
	//https://www.raspberrypi.org/documentation/hardware/raspberrypi/revision-codes/README.md
	pipe = popen("cat /sys/firmware/devicetree/base/model", "r");
	if(fgets(buffer, 128, pipe) != NULL) IUSaveText(&SysInfoT[0], buffer);
	pclose(pipe);

	//update Hostname
	pipe = popen("hostname", "r");
	if(fgets(buffer, 128, pipe) != NULL) IUSaveText(&SysInfoT[4], buffer);
	pclose(pipe);

	//update Local IP
	pipe = popen("hostname -I|awk -F' '  '{print $1}'|xargs", "r");
	if(fgets(buffer, 128, pipe) != NULL) IUSaveText(&SysInfoT[5], buffer);
	pclose(pipe);

	//update Public IP
	pipe = popen("wget -qO- http://ipecho.net/plain|xargs", "r");
	if(fgets(buffer, 128, pipe) != NULL) IUSaveText(&SysInfoT[6], buffer);
	pclose(pipe);

	// Update client
	IDSetText(&SysInfoTP, NULL);

	return true;
}

bool IndiAstrolink4Pi::Disconnect()
{
	// Close GPIO
	gpiod_chip_close(chip);

	// Stop timers
	IERmTimer(stepperStandbyID);
	IERmTimer(updateTemperatureID);
	IERmTimer(temperatureCompensationID);

	IDMessage(getDeviceName(), "AstroLink 4 Pi disconnected successfully.");
	return true;
}

//////////////////////////////////////////////////////////////////////
/// Overrides
//////////////////////////////////////////////////////////////////////
bool IndiAstrolink4Pi::initProperties()
{
    INDI::DefaultDevice::initProperties();

    IUFillText(&SysTimeT[0],"LOCAL_TIME","Local Time",NULL);
	IUFillText(&SysTimeT[1],"UTC_OFFSET","UTC Offset",NULL);
	IUFillTextVector(&SysTimeTP,SysTimeT,2,getDeviceName(),"SYSTEM_TIME","System Time",MAIN_CONTROL_TAB,IP_RO,60,IPS_IDLE);

	IUFillText(&SysInfoT[0],"HARDWARE","Hardware",NULL);
	IUFillText(&SysInfoT[1],"CPU TEMP","CPU Temp (°C)",NULL);
	IUFillText(&SysInfoT[2],"UPTIME","Uptime (hh:mm)",NULL);
	IUFillText(&SysInfoT[3],"LOAD","Load (1 / 5 / 15 min.)",NULL);
	IUFillText(&SysInfoT[4],"HOSTNAME","Hostname",NULL);
	IUFillText(&SysInfoT[5],"LOCAL_IP","Local IP",NULL);
	IUFillText(&SysInfoT[6],"PUBLIC_IP","Public IP",NULL);
	IUFillText(&SysInfoT[7],"SYS_FAN","System fan [%]",NULL);
	IUFillTextVector(&SysInfoTP,SysInfoT,8,getDeviceName(),"SYSTEM_INFO","System Info",MAIN_CONTROL_TAB,IP_RO,60,IPS_IDLE);

    IUFillSwitch(&SysControlS[0], "SYSCTRL_REBOOT", "Reboot", ISS_OFF);
	IUFillSwitch(&SysControlS[1], "SYSCTRL_SHUTDOWN", "Shutdown", ISS_OFF);
	IUFillSwitchVector(&SysControlSP, SysControlS, 2, getDeviceName(), "SYSCTRL", "System Ctrl", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillSwitch(&SysOpConfirmS[0], "SYSOPCONFIRM_CONFIRM", "Yes", ISS_OFF);
	IUFillSwitch(&SysOpConfirmS[1], "SYSOPCONFIRM_CANCEL", "No", ISS_OFF);
	IUFillSwitchVector(&SysOpConfirmSP, SysOpConfirmS, 2, getDeviceName(), "SYSOPCONFIRM", "Continue?", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Focuser Resolution
	IUFillSwitch(&FocusResolutionS[0],"FOCUS_RESOLUTION_1","Full Step",ISS_ON);
	IUFillSwitch(&FocusResolutionS[1],"FOCUS_RESOLUTION_2","Half Step",ISS_OFF);
	IUFillSwitchVector(&FocusResolutionSP,FocusResolutionS,2,getDeviceName(),"FOCUS_RESOLUTION","Resolution",MAIN_CONTROL_TAB,IP_RW,ISR_1OFMANY,0,IPS_IDLE);

    // Focuser Info
	IUFillNumber(&FocuserInfoN[0], "CFZ_STEP_ACT", "Step Size (μm)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[1], "CFZ", "Critical Focus Zone (μm)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[2], "STEPS_PER_CFZ", "Steps / Critical Focus Zone", "%0.0f", 0, 1000, 1, 0);
	IUFillNumberVector(&FocuserInfoNP, FocuserInfoN, 3, getDeviceName(), "FOCUSER_PARAMETERS", "Focuser Info", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

	// Step delay setting
	IUFillNumber(&FocusStepDelayN[0], "FOCUS_STEPDELAY_VALUE", "milliseconds", "%0.0f", 1, 100, 1, 1);
	IUFillNumberVector(&FocusStepDelayNP, FocusStepDelayN, 1, getDeviceName(), "FOCUS_STEPDELAY", "Step Delay", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Backlash setting
	IUFillNumber(&FocusBacklashN[0], "FOCUS_BACKLASH_VALUE", "steps", "%0.0f", 0, 1000, 10, 0);
	IUFillNumberVector(&FocusBacklashNP, FocusBacklashN, 1, getDeviceName(), "FOCUS_BACKLASH", "Backlash", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Reset absolute possition
	IUFillSwitch(&ResetAbsPosS[0],"RESET_ABS","Purge",ISS_OFF);
	IUFillSwitchVector(&ResetAbsPosSP,ResetAbsPosS,1,getDeviceName(),"RESET_ABS_SW","Saved Position",OPTIONS_TAB,IP_RW,ISR_1OFMANY,0,IPS_IDLE);

	// Active telescope setting
	IUFillText(&ActiveTelescopeT[0], "ACTIVE_TELESCOPE_NAME", "Telescope", "Telescope Simulator");
	IUFillTextVector(&ActiveTelescopeTP, ActiveTelescopeT, 1, getDeviceName(), "ACTIVE_TELESCOPE", "Snoop devices", OPTIONS_TAB,IP_RW, 0, IPS_IDLE);

	// Maximum focuser travel
	IUFillNumber(&FocuserTravelN[0], "FOCUSER_TRAVEL_VALUE", "mm", "%0.0f", 10, 200, 10, 10);
	IUFillNumberVector(&FocuserTravelNP, FocuserTravelN, 1, getDeviceName(), "FOCUSER_TRAVEL", "Max Travel", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Focuser temperature
	IUFillNumber(&FocusTemperatureN[0], "FOCUS_TEMPERATURE_VALUE", "°C", "%0.2f", -50, 50, 1, 0);
	IUFillNumberVector(&FocusTemperatureNP, FocusTemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

	// Temperature Coefficient
	IUFillNumber(&TemperatureCoefN[0], "steps/°C", "", "%.1f", -1000, 1000, 1, 0);
	IUFillNumberVector(&TemperatureCoefNP, TemperatureCoefN, 1, getDeviceName(), "Temperature Coefficient", "", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

	// Compensate for temperature
	IUFillSwitch(&TemperatureCompensateS[0], "Enable", "", ISS_OFF);
	IUFillSwitch(&TemperatureCompensateS[1], "Disable", "", ISS_ON);
	IUFillSwitchVector(&TemperatureCompensateSP, TemperatureCompensateS, 2, getDeviceName(), "Temperature Compensate", "", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Snooping params
	IUFillNumber(&ScopeParametersN[0], "TELESCOPE_APERTURE", "Aperture (mm)", "%g", 10, 5000, 0, 0.0);
	IUFillNumber(&ScopeParametersN[1], "TELESCOPE_FOCAL_LENGTH", "Focal Length (mm)", "%g", 10, 10000, 0, 0.0);
	IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, ActiveTelescopeT[0].text, "TELESCOPE_INFO", "Scope Properties", OPTIONS_TAB, IP_RW, 60, IPS_OK);

	// initial values at resolution 1/1
	FocusMaxPosN[0].min = 1000;
	FocusMaxPosN[0].max = 100000;
	FocusMaxPosN[0].step = 1000;
	FocusMaxPosN[0].value = 10000;

	FocusRelPosN[0].min = 0;
	FocusRelPosN[0].max = 1000;
	FocusRelPosN[0].step = 100;
	FocusRelPosN[0].value = 100;

	FocusAbsPosN[0].min = 0;
	FocusAbsPosN[0].max = FocusMaxPosN[0].value;
	FocusAbsPosN[0].step = (int) FocusAbsPosN[0].max / 100;

	FocusMotionS[FOCUS_OUTWARD].s = ISS_ON;
	FocusMotionS[FOCUS_INWARD].s = ISS_OFF;
	IDSetSwitch(&FocusMotionSP, nullptr);

	addDebugControl ();
	addConfigurationControl();
	removeProperty("POLLING_PERIOD", nullptr);        

    return true;
}

bool IndiAstrolink4Pi::updateProperties()
{
	// Call parent update properties first
	INDI::DefaultDevice::updateProperties();

	if (isConnected())
	{
		defineProperty(&SysTimeTP);
		defineProperty(&SysInfoTP);
		defineProperty(&SysControlSP);

		defineProperty(&ActiveTelescopeTP);
		defineProperty(&FocuserTravelNP);
		defineProperty(&FocusMotionSP);
		defineProperty(&FocusResolutionSP);
		defineProperty(&FocuserInfoNP);
		defineProperty(&FocusStepDelayNP);
		defineProperty(&FocusBacklashNP);
		defineProperty(&ResetAbsPosSP);

		IDSnoopDevice(ActiveTelescopeT[0].text, "TELESCOPE_INFO");        

		if (readDS18B20())
		{
			defineProperty(&FocusTemperatureNP);
			defineProperty(&TemperatureCoefNP);
			defineProperty(&TemperatureCompensateSP);
			readDS18B20(); // update immediately
			lastTemperature = FocusTemperatureN[0].value; // init last temperature
			IERmTimer(updateTemperatureID);
			updateTemperatureID = IEAddTimer(TEMPERATURE_UPDATE_TIMEOUT, updateTemperatureHelper, this); // set temperature update timer
			IERmTimer(temperatureCompensationID);
			temperatureCompensationID = IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this); // set temperature compensation timer
			DEBUGF(INDI::Logger::DBG_WARNING, "Comp timer %i", temperatureCompensationID);
		}        
	}
	else
	{
		// We're disconnected
		deleteProperty(SysTimeTP.name);
		deleteProperty(SysInfoTP.name);
		deleteProperty(SysControlSP.name);
		deleteProperty(ActiveTelescopeTP.name);
		deleteProperty(FocuserTravelNP.name);
		deleteProperty(FocusMotionSP.name);
		deleteProperty(FocusResolutionSP.name);
		deleteProperty(FocuserInfoNP.name);
		deleteProperty(FocusStepDelayNP.name);
		deleteProperty(FocusBacklashNP.name);
		deleteProperty(ResetAbsPosSP.name);
		deleteProperty(FocusTemperatureNP.name);
		deleteProperty(TemperatureCoefNP.name);
		deleteProperty(TemperatureCompensateSP.name);        
	}
	return true;
}

void IndiAstrolink4Pi::ISGetProperties(const char *dev)
{
	INDI::DefaultDevice::ISGetProperties(dev);
}

bool IndiAstrolink4Pi::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	// first we check if it's for our device
	if(!strcmp(dev,getDeviceName()))
	{
		// handle focus maximum position
		if (!strcmp(name, FocusMaxPosNP.name))
		{
			IUUpdateNumber(&FocusMaxPosNP,values,names,n);

			FocusAbsPosN[0].max = FocusMaxPosN[0].value;
			IUUpdateMinMax(&FocusAbsPosNP); // This call is not INDI protocol compliant

			FocusAbsPosNP.s=IPS_OK;
			IDSetNumber(&FocusMaxPosNP, nullptr);
			getFocuserInfo();
			return true;
		}

		// handle focus absolute position
		if (!strcmp(name, FocusAbsPosNP.name))
		{
			int newPos = (int) values[0];

			if ( MoveAbsFocuser(newPos) == IPS_OK )
			{
				IUUpdateNumber(&FocusAbsPosNP,values,names,n);
				FocusAbsPosNP.s=IPS_OK;
				IDSetNumber(&FocusAbsPosNP, nullptr);
				return true;
			} else {
				return false;
			}
		}

		// handle focus relative position
		if (!strcmp(name, FocusRelPosNP.name))
		{
			IUUpdateNumber(&FocusRelPosNP,values,names,n);

			//FOCUS_INWARD
			if ( FocusMotionS[0].s == ISS_ON )
				FocusRelPosNP.s = MoveRelFocuser(FOCUS_INWARD, FocusRelPosN[0].value);

			//FOCUS_OUTWARD
			if ( FocusMotionS[1].s == ISS_ON )
				FocusRelPosNP.s = MoveRelFocuser(FOCUS_OUTWARD, FocusRelPosN[0].value);

			IDSetNumber(&FocusRelPosNP, nullptr);
			return true;
		}

		// handle focus backlash
		if (!strcmp(name, FocusBacklashNP.name))
		{
			IUUpdateNumber(&FocusBacklashNP,values,names,n);
			FocusBacklashNP.s=IPS_BUSY;
			IDSetNumber(&FocusBacklashNP, nullptr);
			FocusBacklashNP.s=IPS_OK;
			IDSetNumber(&FocusBacklashNP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Backlash set to %0.0f steps.", FocusBacklashN[0].value);
			return true;
		}

		// handle focus step delay
		if (!strcmp(name, FocusStepDelayNP.name))
		{
			IUUpdateNumber(&FocusStepDelayNP,values,names,n);
			FocusStepDelayNP.s=IPS_BUSY;
			IDSetNumber(&FocusStepDelayNP, nullptr);
			FocusStepDelayNP.s=IPS_OK;
			IDSetNumber(&FocusStepDelayNP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Step delay set to %0.0f ms.", FocusStepDelayN[0].value);
			return true;
		}

		// handle focuser travel
		if (!strcmp(name, FocuserTravelNP.name))
		{
			IUUpdateNumber(&FocuserTravelNP,values,names,n);
			getFocuserInfo();
			FocuserTravelNP.s=IPS_OK;
			IDSetNumber(&FocuserTravelNP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Maximum focuser travel set to %0.0f mm", FocuserTravelN[0].value);
			return true;
		}

		// handle temperature coefficient
		if (!strcmp(name, TemperatureCoefNP.name))
		{
			IUUpdateNumber(&TemperatureCoefNP,values,names,n);
			TemperatureCoefNP.s=IPS_OK;
			IDSetNumber(&TemperatureCoefNP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Temperature coefficient set to %0.1f steps/°C", TemperatureCoefN[0].value);
			return true;
		}
	}

	return INDI::Focuser::ISNewNumber(dev,name,values,names,n);
}

bool IndiAstrolink4Pi::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
		// handle system control
		if (!strcmp(name, SysControlSP.name))
		{
			IUUpdateSwitch(&SysControlSP, states, names, n);

			if ( SysControlS[0].s == ISS_ON )
			{
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi device is set to REBOOT. Confirm or Cancel operation.");
				SysControlSP.s = IPS_BUSY;
				IDSetSwitch(&SysControlSP, NULL);
				
				// confirm switch
				defineProperty(&SysOpConfirmSP);

				return true;
			}
			if ( SysControlS[1].s == ISS_ON )
			{
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi device is set to SHUT DOWN. Confirm or Cancel operation.");
				SysControlSP.s = IPS_BUSY;
				IDSetSwitch(&SysControlSP, NULL);

				// confirm switch
				defineProperty(&SysOpConfirmSP);

				return true;
			}
		}

		// handle system control confirmation
		if (!strcmp(name, SysOpConfirmSP.name))
		{
			IUUpdateSwitch(&SysOpConfirmSP, states, names, n);

			if ( SysOpConfirmS[0].s == ISS_ON )
			{
				SysOpConfirmSP.s = IPS_IDLE;
				IDSetSwitch(&SysOpConfirmSP, NULL);
				SysOpConfirmS[0].s = ISS_OFF;
				IDSetSwitch(&SysOpConfirmSP, NULL);

				// execute system operation
				if (SysControlS[0].s == ISS_ON)
				{
					DEBUG(INDI::Logger::DBG_SESSION, "System operation confirmed. System is going to REBOOT now");
					FILE* pipe;
					char buffer[512];
					pipe = popen("sudo reboot", "r");
					if(fgets(buffer, 512, pipe) == NULL)
					{
						DEBUGF(INDI::Logger::DBG_SESSION, "Failed open sudo reboot", buffer);
					}
					else
					{
						DEBUGF(INDI::Logger::DBG_SESSION, "System output: %s", buffer);
					}
					pclose(pipe);
				}
				if (SysControlS[1].s == ISS_ON)
				{
					DEBUG(INDI::Logger::DBG_SESSION, "System operation confirmed. System is going to SHUT DOWN now");
					FILE* pipe;
					char buffer[512];
					pipe = popen("sudo poweroff", "r");
					if(fgets(buffer, 512, pipe) == NULL)
					{
						DEBUGF(INDI::Logger::DBG_SESSION, "Failed open sudo poweroff", buffer);
					}
					else
					{
						DEBUGF(INDI::Logger::DBG_SESSION, "System output: %s", buffer);
					}
					pclose(pipe);					
				}

				// reset system control buttons
				SysControlSP.s = IPS_IDLE;
				IDSetSwitch(&SysControlSP, NULL);
				SysControlS[0].s = ISS_OFF;
				SysControlS[1].s = ISS_OFF;
				IDSetSwitch(&SysControlSP, NULL);

				deleteProperty(SysOpConfirmSP.name);
				return true;
			}

			if ( SysOpConfirmS[1].s == ISS_ON )
			{
				DEBUG(INDI::Logger::DBG_SESSION, "System operation canceled.");
				SysOpConfirmSP.s = IPS_IDLE;
				IDSetSwitch(&SysOpConfirmSP, NULL);
				SysOpConfirmS[1].s = ISS_OFF;
				IDSetSwitch(&SysOpConfirmSP, NULL);

				// reset system control buttons
				SysControlSP.s = IPS_IDLE;
				IDSetSwitch(&SysControlSP, NULL);
				SysControlS[0].s = ISS_OFF;
				SysControlS[1].s = ISS_OFF;
				IDSetSwitch(&SysControlSP, NULL);

				deleteProperty(SysOpConfirmSP.name);
				return true;
			}
		}
	
	    // handle focus resolution
	    if(!strcmp(name, FocusResolutionSP.name))
	    {
			int last_resolution = resolution;
			IUFindOnSwitchIndex(&FocusResolutionSP);

			IUUpdateSwitch(&FocusResolutionSP, states, names, n);

			//Resolution 1/1
			if ( FocusResolutionS[0].s == ISS_ON )
				resolution = 1;

			//Resolution 1/2
			if ( FocusResolutionS[1].s == ISS_ON )
				resolution = 2;

			// Adjust position to a step in lower resolution
			int position_adjustment = last_resolution * (FocusAbsPosN[0].value / last_resolution - (int) FocusAbsPosN[0].value / last_resolution);
			if ( resolution < last_resolution && position_adjustment > 0 )
			{
				if ( (float) position_adjustment / last_resolution < 0.5)
				{
					position_adjustment *= -1;
				} else {
					position_adjustment = last_resolution - position_adjustment;
				}
				DEBUGF(INDI::Logger::DBG_SESSION, "Focuser position adjusted by %d steps at 1/%d resolution to sync with 1/%d resolution.", position_adjustment, last_resolution, resolution);
				MoveAbsFocuser(FocusAbsPosN[0].value + position_adjustment);
			}

			SetResolution(resolution);

			// update values based on resolution
			FocusRelPosN[0].min = (int) FocusRelPosN[0].min * resolution / last_resolution;
			FocusRelPosN[0].max = (int) FocusRelPosN[0].max * resolution / last_resolution;
			FocusRelPosN[0].step = (int) FocusRelPosN[0].step * resolution / last_resolution;
			FocusRelPosN[0].value = (int) FocusRelPosN[0].value * resolution / last_resolution;
			IDSetNumber(&FocusRelPosNP, nullptr);
			IUUpdateMinMax(&FocusRelPosNP);

			FocusAbsPosN[0].max = (int) FocusAbsPosN[0].max * resolution / last_resolution;
			FocusAbsPosN[0].step = (int) FocusAbsPosN[0].step * resolution / last_resolution;
			FocusAbsPosN[0].value = (int) FocusAbsPosN[0].value * resolution / last_resolution;
			IDSetNumber(&FocusAbsPosNP, nullptr);
			IUUpdateMinMax(&FocusAbsPosNP);

			FocusMaxPosN[0].min = (int) FocusMaxPosN[0].min * resolution / last_resolution;
			FocusMaxPosN[0].max = (int) FocusMaxPosN[0].max * resolution / last_resolution;
			FocusMaxPosN[0].step = (int) FocusMaxPosN[0].step * resolution / last_resolution;
			FocusMaxPosN[0].value = (int) FocusMaxPosN[0].value * resolution / last_resolution;
			IDSetNumber(&FocusMaxPosNP, nullptr);
			IUUpdateMinMax(&FocusMaxPosNP);

			PresetN[0].value = (int) PresetN[0].value * resolution / last_resolution;
			PresetN[1].value = (int) PresetN[1].value * resolution / last_resolution;
			PresetN[2].value = (int) PresetN[2].value * resolution / last_resolution;
			IDSetNumber(&PresetNP, nullptr);

			getFocuserInfo();

			FocusResolutionSP.s = IPS_OK;
			IDSetSwitch(&FocusResolutionSP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Focuser resolution set to 1/%d.", resolution);
			return true;
		}

	    // handle reset absolute position
	    if(!strcmp(name, ResetAbsPosSP.name))
	    {
			IUResetSwitch(&ResetAbsPosSP);

			//set absolute position to zero and save to file
			FocusAbsPosN[0].value = 0;
			IDSetNumber(&FocusAbsPosNP, nullptr);
			savePosition(0);

			DEBUG(INDI::Logger::DBG_SESSION, "Absolute Position reset to 0.");

			ResetAbsPosSP.s = IPS_IDLE;
			IDSetSwitch(&ResetAbsPosSP, nullptr);
			return true;
		}

	    // handle temperature compensation
	    if(!strcmp(name, TemperatureCompensateSP.name))
	    {
			IUUpdateSwitch(&TemperatureCompensateSP, states, names, n);

			if ( TemperatureCompensateS[0].s == ISS_ON)
			{
				temperatureCompensationID = IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this);
				TemperatureCompensateSP.s = IPS_OK;
				DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation ENABLED.");
			}

			if ( TemperatureCompensateS[1].s == ISS_ON)
			{
				IERmTimer(temperatureCompensationID);
				TemperatureCompensateSP.s = IPS_IDLE;
				DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation DISABLED.");
			}

			IDSetSwitch(&TemperatureCompensateSP, nullptr);
			return true;
		}    
    }
	return INDI::DefaultDevice::ISNewSwitch (dev, name, states, names, n);
}

bool IndiAstrolink4Pi::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
		// handle active devices
		if (!strcmp(name, ActiveTelescopeTP.name))
		{
				IUUpdateText(&ActiveTelescopeTP,texts,names,n);

				IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, ActiveTelescopeT[0].text, "TELESCOPE_INFO", "Scope Properties", OPTIONS_TAB, IP_RW, 60, IPS_OK);
				IDSnoopDevice(ActiveTelescopeT[0].text, "TELESCOPE_INFO");

				ActiveTelescopeTP.s=IPS_OK;
				IDSetText(&ActiveTelescopeTP, nullptr);
				DEBUGF(INDI::Logger::DBG_SESSION, "Active telescope set to %s.", ActiveTelescopeT[0].text);
				return true;
		}
	}    
	return INDI::DefaultDevice::ISNewText (dev, name, texts, names, n);
}

bool IndiAstrolink4Pi::ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
	return INDI::DefaultDevice::ISNewBLOB (dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool IndiAstrolink4Pi::ISSnoopDevice (XMLEle *root)
{
	if (IUSnoopNumber(root, &ScopeParametersNP) == 0)
	{
		getFocuserInfo();
		DEBUGF(INDI::Logger::DBG_DEBUG, "Telescope parameters: %0.0f, %0.0f.", ScopeParametersN[0].value, ScopeParametersN[1].value);
		return true;
	}

	return INDI::Focuser::ISSnoopDevice(root);
}

bool IndiAstrolink4Pi::saveConfigItems(FILE *fp)
{
	IUSaveConfigSwitch(fp, &FocusResolutionSP);
	IUSaveConfigSwitch(fp, &FocusReverseSP);
	IUSaveConfigSwitch(fp, &TemperatureCompensateSP);
	IUSaveConfigNumber(fp, &FocusMaxPosNP);
	IUSaveConfigNumber(fp, &FocusStepDelayNP);
	IUSaveConfigNumber(fp, &FocusBacklashNP);
	IUSaveConfigNumber(fp, &FocuserTravelNP);
	IUSaveConfigNumber(fp, &PresetNP);
	IUSaveConfigNumber(fp, &TemperatureCoefNP);
	IUSaveConfigText(fp, &ActiveTelescopeTP);
	return true;
}

void IndiAstrolink4Pi::stepperRun()
{
	if(backlashTicksRemaining <= 0 && ticksRemaining <= 0)
	{
		//All movement completed/aborted
		//save position to file
		savePosition((int) FocusAbsPosN[0].value * MAX_RESOLUTION / resolution); // always save at MAX_RESOLUTION

		// update abspos value and status
		DEBUGF(INDI::Logger::DBG_SESSION, "Focuser at the position %0.0f.", FocusAbsPosN[0].value);

		FocusAbsPosNP.s = IPS_OK;
		IDSetNumber(&FocusAbsPosNP, nullptr);

		// reset last temperature
		lastTemperature = FocusTemperatureN[0].value; // register last temperature

		// set motor standby timer
		IERmTimer(stepperStandbyID);
		stepperStandbyID = IEAddTimer(STEPPER_STANDBY_TIMEOUT, stepperStandbyHelper, this);

		return;
	}

	int motorDirection = lastDirection;

	// handle Reverse Motion
	if (FocusReverseS[INDI_ENABLED].s == ISS_ON) 
	{
		motorDirection = -1 * motorDirection;
	}

	bool isBacklash = false;
	if(backlashTicksRemaining > 0)
	{
		isBacklash = true;
	}


	//Move the actual motor
	stepMotor(motorDirection);

	if(isBacklash == false)
	{//Only Count the position change if it is not due to backlash
		// INWARD - count down
		if ( lastDirection == -1 )
			FocusAbsPosN[0].value -= 1;

		// OUTWARD - count up
		if ( lastDirection == 1 )
			FocusAbsPosN[0].value += 1;

		IDSetNumber(&FocusAbsPosNP, nullptr);

		//decrement counter
		ticksRemaining -= 1;
	}
	else
	{//Don't count the backlash position change, just decrement the counter
		backlashTicksRemaining -= 1;
	}

	stepperRunID = IEAddTimer(STEPPER_CYCLE, stepperRunHelper, this);
}

bool IndiAstrolink4Pi::AbortFocuser()
{
	DEBUG(INDI::Logger::DBG_SESSION, "Focuser motion aborted.");
	backlashTicksRemaining = 0;
	ticksRemaining = 0;
	return true;
}

void IndiAstrolink4Pi::stepMotor(int direction)
{
	currentStep = currentStep + direction;

	if (currentStep < 0)
	{
		currentStep = 7;
	}

	if (currentStep > 7)
	{
		currentStep = 0;
	}

	if (resolution == 1)
	{	//Full Step
		gpiod_line_set_value(gpio_a1, fullStep[currentStep][0]);
        gpiod_line_set_value(gpio_a2, fullStep[currentStep][1]);
        gpiod_line_set_value(gpio_b1, fullStep[currentStep][2]);
        gpiod_line_set_value(gpio_b2, fullStep[currentStep][3]);
	}
	else if (resolution == 2)
	{	//Half Step
		gpiod_line_set_value(gpio_a1, halfStep[currentStep][0]);
        gpiod_line_set_value(gpio_a2, halfStep[currentStep][1]);
        gpiod_line_set_value(gpio_b1, halfStep[currentStep][2]);
        gpiod_line_set_value(gpio_b2, halfStep[currentStep][3]);
	}
}

IPState IndiAstrolink4Pi::MoveRelFocuser(FocusDirection dir, int ticks)
{
	int targetTicks = FocusAbsPosN[0].value + ((int32_t)ticks * (dir == FOCUS_INWARD ? -1 : 1));
	return MoveAbsFocuser(targetTicks);
}

IPState IndiAstrolink4Pi::MoveAbsFocuser(int targetTicks)
{
	if(backlashTicksRemaining > 0 || ticksRemaining > 0)
    {
        DEBUG(INDI::Logger::DBG_WARNING, "Focuser movement still in progress.");
        return IPS_BUSY;
	}

	if (targetTicks < FocusAbsPosN[0].min || targetTicks > FocusAbsPosN[0].max)
	{
		DEBUG(INDI::Logger::DBG_WARNING, "Requested position is out of range.");
		return IPS_ALERT;
	}

	if (targetTicks == FocusAbsPosN[0].value)
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Already at the requested position.");
		return IPS_OK;
	}

	// set focuser busy
	FocusAbsPosNP.s = IPS_BUSY;
	IDSetNumber(&FocusAbsPosNP, nullptr);

	// check last motion direction for backlash triggering
	int lastdir = lastDirection;

	// set direction
	const char* direction;
	int newDirection;
	if (targetTicks > FocusAbsPosN[0].value)
	{
		// OUTWARD
		direction = "OUTWARD";
		newDirection = 1;
	} else {
		// INWARD
		direction = "INWARD";
		newDirection = -1;
	}

	lastDirection = newDirection;

	// if direction changed do backlash adjustment
	if ( lastdir != 0 && newDirection != lastdir && FocusBacklashN[0].value != 0)
	{
		DEBUGF(INDI::Logger::DBG_SESSION, "Backlash compensation by %0.0f steps.", FocusBacklashN[0].value);
		backlashTicksRemaining = FocusBacklashN[0].value;
	}
	else
	{
		backlashTicksRemaining = 0;
	}

	// process targetTicks
	ticksRemaining = abs(targetTicks - FocusAbsPosN[0].value);

	DEBUGF(INDI::Logger::DBG_SESSION, "Focuser is moving %s to position %d.", direction, targetTicks);

	SetTimer(FocusStepDelayN[0].value);

	return IPS_BUSY;
}

void IndiAstrolink4Pi::SetResolution(int res)
{
	DEBUGF(INDI::Logger::DBG_SESSION, "Resolution set to 1 / %0.0f.", res);
}

bool IndiAstrolink4Pi::ReverseFocuser(bool enabled)
{
	if (enabled)
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Reverse direction ENABLED.");
	} else {
		DEBUG(INDI::Logger::DBG_SESSION, "Reverse direction DISABLED.");
	}
	return true;
}

int IndiAstrolink4Pi::savePosition(int pos)
{
	FILE * pFile;
	char posFileName[MAXRBUF];
	char buf [100];

	if (getenv("INDICONFIG"))
	{
		snprintf(posFileName, MAXRBUF, "%s.position", getenv("INDICONFIG"));
	} else {
		snprintf(posFileName, MAXRBUF, "%s/.indi/%s.position", getenv("HOME"), getDeviceName());
	}


	if (pos == -1)
	{
		pFile = fopen (posFileName,"r");
		if (pFile == NULL)
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "Failed to open file %s.", posFileName);
			return -1;
		}

		if(fgets (buf , 100, pFile) == NULL)
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "Failed to read file %s.", posFileName);
			return -1;

		}
		else
		{
			pos = atoi (buf);
			DEBUGF(INDI::Logger::DBG_DEBUG, "Reading position %d from %s.", pos, posFileName);
		}
		
	} else {
		pFile = fopen (posFileName,"w");
		if (pFile == NULL)
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "Failed to open file %s.", posFileName);
			return -1;
		}

		sprintf(buf, "%d", pos);
		fputs (buf, pFile);
		DEBUGF(INDI::Logger::DBG_DEBUG, "Writing position %s to %s.", buf, posFileName);
	}

	fclose (pFile);

	return pos;
}

bool IndiAstrolink4Pi::readDS18B20()
{
	DIR *dir;
	struct dirent *dirent;
	char dev[16];      // Dev ID
	char devPath[128]; // Path to device
	char buf[256]; // Data from device
	char temperatureData[6]; // Temp C * 1000 reported by device
	char path[] = "/sys/bus/w1/devices";
	ssize_t numRead;
	float tempC;

	dir = opendir (path);

	// search for --the first-- DS18B20 device
	if (dir != NULL)
	{
		while ((dirent = readdir (dir)))
		{
			// DS18B20 device is family code beginning with 28-
			if (dirent->d_type == DT_LNK && strstr(dirent->d_name, "28-") != NULL)
			{
				strcpy(dev, dirent->d_name);
				break;
			}
		}
		(void) closedir (dir);
	} else {
		DEBUG(INDI::Logger::DBG_WARNING, "Temperature sensor disabled. 1-Wire interface is not available.");
		return false;
	}

	// Assemble path to --the first-- DS18B20 device
	sprintf(devPath, "%s/%s/w1_slave", path, dev);

	// Opening the device's file triggers new reading
	int fd = open(devPath, O_RDONLY);
	if(fd == -1)
	{
		DEBUG(INDI::Logger::DBG_WARNING, "Temperature sensor not available.");
		return false;
	}

	// set busy
	FocusTemperatureNP.s=IPS_BUSY;
	IDSetNumber(&FocusTemperatureNP, nullptr);

	// read sensor output
	while((numRead = read(fd, buf, 256)) > 0);
	close(fd);

	// parse temperature value from sensor output
	strncpy(temperatureData, strstr(buf, "t=") + 2, 5);
	DEBUGF(INDI::Logger::DBG_DEBUG, "Temperature sensor raw output: %s", buf);
	DEBUGF(INDI::Logger::DBG_DEBUG, "Temperature string: %s", temperatureData);

	tempC = strtof(temperatureData, NULL) / 1000;
	// tempF = (tempC / 1000) * 9 / 5 + 32;

	// check if temperature is reasonable
	if(abs(tempC) > 100)
	{
		DEBUG(INDI::Logger::DBG_DEBUG, "Temperature reading out of range.");
		return false;
	}

	FocusTemperatureN[0].value = tempC;

	// set OK
	FocusTemperatureNP.s=IPS_OK;
	IDSetNumber(&FocusTemperatureNP, nullptr);
	DEBUGF(INDI::Logger::DBG_DEBUG, "Temperature: %.2f°C", tempC);

	return true;
}

bool IndiAstrolink4Pi::SyncFocuser(uint32_t ticks)
{
    FocusAbsPosN[0].value = ticks;
    IDSetNumber(&FocusAbsPosNP, nullptr);
    savePosition(ticks);

    DEBUGF(INDI::Logger::DBG_SESSION, "Absolute Position reset to %0.0f", FocusAbsPosN[0].value);

    return true;
}

void IndiAstrolink4Pi::getFocuserInfo()
{
	// https://www.innovationsforesight.com/education/how-much-focus-error-is-too-much/
	float travel_mm = (float) FocuserTravelN[0].value;
	float aperture = (float) ScopeParametersN[0].value;
	float focal = (float) ScopeParametersN[1].value;
	float f_ratio;

	// handle no snooping data from telescope
	if ( aperture * focal != 0 )
	{
		f_ratio = focal / aperture;
	} else {
		f_ratio =  0;
		DEBUG(INDI::Logger::DBG_DEBUG, "No telescope focal length and/or aperture info available.");
	}

	float cfz = 4.88 * 0.520 * pow(f_ratio, 2); // CFZ = 4.88 · λ · f^2
	float step_size = 1000.0 * travel_mm / FocusMaxPosN[0].value;
	float steps_per_cfz = (int) cfz / step_size;

	if ( steps_per_cfz >= 4  )
	{
		FocuserInfoNP.s = IPS_OK;
	}
	else if ( steps_per_cfz > 2 && steps_per_cfz < 4 )
	{
		FocuserInfoNP.s = IPS_BUSY;
	} else {
		FocuserInfoNP.s = IPS_ALERT;
	}

	FocuserInfoN[0].value = step_size;
	FocuserInfoN[1].value = cfz;
	FocuserInfoN[2].value = steps_per_cfz;
	IDSetNumber(&FocuserInfoNP, nullptr);

	DEBUGF(INDI::Logger::DBG_DEBUG, "Focuser Info: %0.2f %0.2f %0.2f.", FocuserInfoN[0].value, FocuserInfoN[1].value, FocuserInfoN[2].value);
}

void IndiAstrolink4Pi::stepperStandbyHelper(void *context)
{
	static_cast<IndiAstrolink4Pi*>(context)->stepperStandby();
}

void IndiAstrolink4Pi::updateTemperatureHelper(void *context)
{
	static_cast<IndiAstrolink4Pi*>(context)->updateTemperature();
}

void IndiAstrolink4Pi::temperatureCompensationHelper(void *context)
{
	static_cast<IndiAstrolink4Pi*>(context)->temperatureCompensation();
}

void IndiAstrolink4Pi::stepperRunnHelper(void *context)
{
	static_cast<IndiAstrolink4Pi*>(context)->stepperRun();
}

void IndiAstrolink4Pi::stepperStandby()
{
	if (!isConnected())
		return;

	gpiod_line_set_value(gpio_a1, 0); 
    gpiod_line_set_value(gpio_a2, 0);
    gpiod_line_set_value(gpio_b1, 0);
    gpiod_line_set_value(gpio_b2, 0);

	DEBUG(INDI::Logger::DBG_SESSION, "Stepper motor going standby.");
}

void IndiAstrolink4Pi::updateTemperature()
{
	if (!isConnected())
		return;

	readDS18B20();
	updateTemperatureID = IEAddTimer(TEMPERATURE_UPDATE_TIMEOUT, updateTemperatureHelper, this);
}

void IndiAstrolink4Pi::temperatureCompensation()
{
	if (!isConnected())
		return;

	if ( TemperatureCompensateS[0].s == ISS_ON && FocusTemperatureN[0].value != lastTemperature )
	{
		float deltaTemperature = FocusTemperatureN[0].value - lastTemperature; // change of temperature from last focuser movement
		float deltaPos = TemperatureCoefN[0].value * deltaTemperature;

		// Move focuser once the compensation is larger than 1/2 CFZ
		if ( abs(deltaPos) > (FocuserInfoN[2].value / 2))
		{
			int thermalAdjustment = round(deltaPos); // adjust focuser by half number of steps to keep it in the center of cfz
			MoveAbsFocuser(FocusAbsPosN[0].value + thermalAdjustment); // adjust focuser position
			lastTemperature = FocusTemperatureN[0].value; // register last temperature
			DEBUGF(INDI::Logger::DBG_SESSION, "Focuser adjusted by %d steps due to temperature change by %0.2f°C", thermalAdjustment, deltaTemperature);
		}
	}

	temperatureCompensationID = IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this);
}