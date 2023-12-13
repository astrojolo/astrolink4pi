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

#include "astrolink4pi.h"

std::unique_ptr<AstroLink4Pi> astroLink4Pi(new AstroLink4Pi());

#define MAX_RESOLUTION 32							 // the highest resolution supported is 1/32 step
#define TEMPERATURE_UPDATE_TIMEOUT (5 * 1000)		 // 5 sec
#define TEMPERATURE_COMPENSATION_TIMEOUT (30 * 1000) // 30 sec
#define SYSTEM_UPDATE_PERIOD 1000
#define POLL_PERIOD 200
#define FAN_PERIOD	(60 * 1000)

#define RP4_GPIO	0
#define RP5_GPIO	4
#define DECAY_PIN 	14
#define EN_PIN 		15
#define M0_PIN 		17
#define M1_PIN 		18
#define M2_PIN 		27
#define RST_PIN 	22
#define STP_PIN 	24
#define DIR_PIN 	23
#define OUT1_PIN 	5
#define OUT2_PIN 	6
#define PWM1_PIN 	26
#define PWM2_PIN 	19
#define HOLD_PIN 	10
#define MOTOR_PWM 	20
#define CHK_IN_PIN 	16
#define FAN_PIN		13

void ISPoll(void *p);

void ISInit()
{
	static int isInit = 0;

	if (isInit == 1)
		return;
	if (astroLink4Pi.get() == 0)
	{
		isInit = 1;
		astroLink4Pi.reset(new AstroLink4Pi());
	}
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
	ISInit();
	astroLink4Pi->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int num)
{
	ISInit();
	astroLink4Pi->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
	astroLink4Pi->ISNewNumber(dev, name, values, names, num);
}

void ISSnoopDevice(XMLEle *root)
{
	astroLink4Pi->ISSnoopDevice(root);
}

AstroLink4Pi::AstroLink4Pi() : FI(this), WI(this)
{
	setVersion(VERSION_MAJOR, VERSION_MINOR);
}

AstroLink4Pi::~AstroLink4Pi()
{
	if (_motionThread.joinable())
	{
		_abort = true;
		_motionThread.join();
	}
}

const char *AstroLink4Pi::getDefaultName()
{
	return (char *)"AstroLink 4 Pi";
}


bool AstroLink4Pi::Connect()
{
	revision = checkRevision();

	pigpioHandle = lgGpiochipOpen(gpioType);
	if (pigpioHandle < 0)
	{
		DEBUGF(INDI::Logger::DBG_ERROR, "Problem initiating AstroLink 4 Pi - GPIO. %d ", pigpioHandle);
		return false;
	}

	if(revision < 4)
	{
		DEBUGF(INDI::Logger::DBG_ERROR, "This INDI driver version works only with AstroLink 4 Pi revision 4 and higer. Revision detected %d", revision);		
		return false;
	}

	lgGpioClaimOutput(pigpioHandle, 0, DECAY_PIN, 0);
	int outs[13] = {EN_PIN, M0_PIN, M1_PIN, M2_PIN, RST_PIN, STP_PIN, DIR_PIN, OUT1_PIN, OUT2_PIN, PWM1_PIN, PWM2_PIN, MOTOR_PWM, HOLD_PIN};
	int lvls[13] = {1, 0, 0, 0, 1, 0, 0, relayState[0], relayState[1], 0, 0, 0, 1};
	// EN_PIN start as disabled
	// RST_PIN start as wake up
	// HOLD_PIN start as disabled

	lgGroupClaimOutput(pigpioHandle, 0, 13, outs, lvls);

	// Lock Relay Labels setting
	RelayLabelsTP.s = IPS_BUSY;
	IDSetText(&RelayLabelsTP, nullptr);

	// Get basic system info
	FILE *pipe;
	char buffer[128];

	// update Hardware
	// https://www.raspberrypi.org/documentation/hardware/raspberrypi/revision-codes/README.md
	pipe = popen("cat /sys/firmware/devicetree/base/model", "r");
	if (fgets(buffer, 128, pipe) != NULL)
		IUSaveText(&SysInfoT[0], buffer);
	pclose(pipe);

	// update Hostname
	pipe = popen("hostname", "r");
	if (fgets(buffer, 128, pipe) != NULL)
		IUSaveText(&SysInfoT[4], buffer);
	pclose(pipe);

	// update Local IP
	pipe = popen("hostname -I|awk -F' '  '{print $1}'|xargs", "r");
	if (fgets(buffer, 128, pipe) != NULL)
		IUSaveText(&SysInfoT[5], buffer);
	pclose(pipe);

	// update Public IP
	pipe = popen("wget -qO- http://ipecho.net/plain|xargs", "r");
	if (fgets(buffer, 128, pipe) != NULL)
		IUSaveText(&SysInfoT[6], buffer);
	pclose(pipe);

	// Update client
	IDSetText(&SysInfoTP, NULL);

	// read last position from file & convert from MAX_RESOLUTION to current resolution
	FocusAbsPosN[0].value = savePosition(-1) != -1 ? (int)savePosition(-1) * resolution / MAX_RESOLUTION : 0;

	// preset resolution
	SetResolution(resolution);

	getFocuserInfo();
	long int currentTime = millis();
	nextTemperatureRead = currentTime + TEMPERATURE_UPDATE_TIMEOUT;
	nextTemperatureCompensation = currentTime + TEMPERATURE_COMPENSATION_TIMEOUT;
	nextSystemRead = currentTime + SYSTEM_UPDATE_PERIOD;
	nextFanUpdate = currentTime + FAN_PERIOD;

	SetTimer(POLL_PERIOD);
	setCurrent(true);

	DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi connected successfully.");

	return true;
}

bool AstroLink4Pi::Disconnect()
{
	lgGpioWrite(pigpioHandle, HOLD_PIN, 1);
	lgGpioWrite(pigpioHandle, RST_PIN, 0);							// sleep
	int enabledState = lgGpioWrite(pigpioHandle, EN_PIN, 1);		// make disabled

	if (enabledState != 0)
	{
		DEBUGF(INDI::Logger::DBG_ERROR, "Cannot set GPIO line %i to disable stepper motor driver. Focusing motor may still be powered.", EN_PIN);
	}
	else
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Focusing motor power disabled.");
	}

	lgGpioFree(pigpioHandle, DECAY_PIN);
	lgGpioFree(pigpioHandle, FAN_PIN);
	lgGroupFree(pigpioHandle, EN_PIN);
	lgGpiochipClose(pigpioHandle);

	// Unlock Relay Labels setting
	RelayLabelsTP.s = IPS_IDLE;
	IDSetText(&RelayLabelsTP, nullptr);

	DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi disconnected successfully.");

	return true;
}

bool AstroLink4Pi::initProperties()
{
	INDI::DefaultDevice::initProperties();

	setDriverInterface(AUX_INTERFACE | FOCUSER_INTERFACE | WEATHER_INTERFACE);

	FI::SetCapability(FOCUSER_CAN_ABS_MOVE |
					  FOCUSER_CAN_REL_MOVE |
					  FOCUSER_CAN_REVERSE |
					  FOCUSER_CAN_SYNC |
					  FOCUSER_CAN_ABORT |
					  FOCUSER_HAS_BACKLASH);

	FI::initProperties(FOCUS_TAB);
	WI::initProperties(ENVIRONMENT_TAB, ENVIRONMENT_TAB);

	RefreshSP.setPermission(IP_RO);	RefreshSP.setState(IPS_IDLE);
	UpdatePeriodNP.setPermission(IP_RO); UpdatePeriodNP.setState(IPS_IDLE);
	double values[1] = {3.0}; const char* names[1] = {"PERIOD"}; UpdatePeriodNP.update(values, names, 1);

	addDebugControl();
	// addSimulationControl();
	addConfigurationControl();

	// Focuser Resolution
	IUFillSwitch(&FocusResolutionS[0], "FOCUS_RESOLUTION_1", "Full Step", ISS_ON);
	IUFillSwitch(&FocusResolutionS[1], "FOCUS_RESOLUTION_2", "Half Step", ISS_OFF);
	IUFillSwitch(&FocusResolutionS[2], "FOCUS_RESOLUTION_4", "1/4 STEP", ISS_OFF);
	IUFillSwitch(&FocusResolutionS[3], "FOCUS_RESOLUTION_8", "1/8 STEP", ISS_OFF);
	IUFillSwitch(&FocusResolutionS[4], "FOCUS_RESOLUTION_16", "1/16 STEP", ISS_OFF);
	IUFillSwitch(&FocusResolutionS[5], "FOCUS_RESOLUTION_32", "1/32 STEP", ISS_OFF);
	IUFillSwitchVector(&FocusResolutionSP, FocusResolutionS, 6, getDeviceName(), "FOCUS_RESOLUTION", "Resolution", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Focuser motor hold
	IUFillSwitch(&FocusHoldS[0], "FOCUS_HOLD_0", "0%", ISS_ON);
	IUFillSwitch(&FocusHoldS[1], "FOCUS_HOLD_1", "20%", ISS_OFF);
	IUFillSwitch(&FocusHoldS[2], "FOCUS_HOLD_2", "40%", ISS_OFF);
	IUFillSwitch(&FocusHoldS[3], "FOCUS_HOLD_3", "60%", ISS_OFF);
	IUFillSwitch(&FocusHoldS[4], "FOCUS_HOLD_4", "80%", ISS_OFF);
	IUFillSwitch(&FocusHoldS[5], "FOCUS_HOLD_5", "100%", ISS_OFF);
	IUFillSwitchVector(&FocusHoldSP, FocusHoldS, 6, getDeviceName(), "FOCUS_HOLD", "Hold power", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Step delay setting
	IUFillNumber(&FocusStepDelayN[0], "FOCUS_STEPDELAY_VALUE", "microseconds", "%0.0f", 200, 20000, 1, 2000);
	IUFillNumberVector(&FocusStepDelayNP, FocusStepDelayN, 1, getDeviceName(), "FOCUS_STEPDELAY", "Step Delay", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	IUFillNumber(&PWMcycleN[0], "PWMcycle", "PWM freq. [Hz]", "%0.0f", 10, 1000, 10, 20);
	IUFillNumberVector(&PWMcycleNP, PWMcycleN, 1, getDeviceName(), "PWMCYCLE", "PWM frequency", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Focuser temperature
	IUFillNumber(&FocusTemperatureN[0], "FOCUS_TEMPERATURE_VALUE", "°C", "%0.2f", -50, 50, 1, 0);
	IUFillNumberVector(&FocusTemperatureNP, FocusTemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

	// Temperature Coefficient
	IUFillNumber(&TemperatureCoefN[0], "steps/°C", "", "%.1f", -1000, 1000, 1, 0);
	IUFillNumberVector(&TemperatureCoefNP, TemperatureCoefN, 1, getDeviceName(), "Temperature Coefficient", "", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Compensate for temperature
	IUFillSwitch(&TemperatureCompensateS[0], "Enable", "", ISS_OFF);
	IUFillSwitch(&TemperatureCompensateS[1], "Disable", "", ISS_ON);
	IUFillSwitchVector(&TemperatureCompensateSP, TemperatureCompensateS, 2, getDeviceName(), "Temperature Compensate", "", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Focuser Info
	IUFillNumber(&FocuserInfoN[0], "CFZ_STEP_ACT", "Step Size (μm)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[1], "CFZ", "Critical Focus Zone (μm)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[2], "STEPS_PER_CFZ", "Steps / Critical Focus Zone", "%0.0f", 0, 1000, 1, 0);
	IUFillNumberVector(&FocuserInfoNP, FocuserInfoN, 3, getDeviceName(), "FOCUSER_PARAMETERS", "Focuser Info", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

	// Maximum focuser travel
	IUFillNumber(&FocuserTravelN[0], "FOCUSER_TRAVEL_VALUE", "mm", "%0.0f", 10, 200, 10, 10);
	IUFillNumberVector(&FocuserTravelNP, FocuserTravelN, 1, getDeviceName(), "FOCUSER_TRAVEL", "Max Travel", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Active telescope setting
	IUFillText(&ActiveTelescopeT[0], "ACTIVE_TELESCOPE_NAME", "Telescope", "Telescope Simulator");
	IUFillTextVector(&ActiveTelescopeTP, ActiveTelescopeT, 1, getDeviceName(), "ACTIVE_TELESCOPE", "Snoop devices", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Snooping params
	IUFillNumber(&ScopeParametersN[0], "TELESCOPE_APERTURE", "Aperture (mm)", "%g", 10, 5000, 0, 0.0);
	IUFillNumber(&ScopeParametersN[1], "TELESCOPE_FOCAL_LENGTH", "Focal Length (mm)", "%g", 10, 10000, 0, 0.0);
	IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, ActiveTelescopeT[0].text, "TELESCOPE_INFO", "Scope Properties", OPTIONS_TAB, IP_RW, 60, IPS_OK);

	IUFillText(&SysTimeT[0], "LOCAL_TIME", "Local Time", NULL);
	IUFillText(&SysTimeT[1], "UTC_OFFSET", "UTC Offset", NULL);
	IUFillTextVector(&SysTimeTP, SysTimeT, 2, getDeviceName(), "SYSTEM_TIME", "System Time", SYSTEM_TAB, IP_RO, 60, IPS_IDLE);

	IUFillText(&SysInfoT[0], "HARDWARE", "Hardware", NULL);
	IUFillText(&SysInfoT[1], "CPU TEMP", "CPU Temp (°C)", NULL);
	IUFillText(&SysInfoT[2], "UPTIME", "Uptime (hh:mm)", NULL);
	IUFillText(&SysInfoT[3], "LOAD", "Load (1 / 5 / 15 min.)", NULL);
	IUFillText(&SysInfoT[4], "HOSTNAME", "Hostname", NULL);
	IUFillText(&SysInfoT[5], "LOCAL_IP", "Local IP", NULL);
	IUFillText(&SysInfoT[6], "PUBLIC_IP", "Public IP", NULL);
	IUFillTextVector(&SysInfoTP, SysInfoT, 7, getDeviceName(), "SYSTEM_INFO", "System Info", SYSTEM_TAB, IP_RO, 60, IPS_IDLE);

	IUFillSwitch(&SysControlS[0], "SYSCTRL_REBOOT", "Reboot", ISS_OFF);
	IUFillSwitch(&SysControlS[1], "SYSCTRL_SHUTDOWN", "Shutdown", ISS_OFF);
	IUFillSwitchVector(&SysControlSP, SysControlS, 2, getDeviceName(), "SYSCTRL", "System Ctrl", SYSTEM_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillSwitch(&SysOpConfirmS[0], "SYSOPCONFIRM_CONFIRM", "Yes", ISS_OFF);
	IUFillSwitch(&SysOpConfirmS[1], "SYSOPCONFIRM_CANCEL", "No", ISS_OFF);
	IUFillSwitchVector(&SysOpConfirmSP, SysOpConfirmS, 2, getDeviceName(), "SYSOPCONFIRM", "Continue?", SYSTEM_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillText(&RelayLabelsT[0], "RELAYLABEL01", "OUT 1", "OUT 1");
	IUFillText(&RelayLabelsT[1], "RELAYLABEL02", "OUT 2", "OUT 2");
	IUFillText(&RelayLabelsT[2], "RELAYLABEL03", "PWM 1", "PWM 1");
	IUFillText(&RelayLabelsT[3], "RELAYLABEL04", "PWM 2", "PWM 2");
	IUFillTextVector(&RelayLabelsTP, RelayLabelsT, 4, getDeviceName(), "RELAYLABELS", "Relay Labels", OPTIONS_TAB, IP_RW, 60, IPS_IDLE);

	// Load options before connecting
	// load config before defining switches
	defineProperty(&RelayLabelsTP);
	loadConfig();

	IUFillNumber(&StepperCurrentN[0], "STEPPER_CURRENT", "mA", "%0.0f", 200, 2000, 50, 400);
	IUFillNumberVector(&StepperCurrentNP, StepperCurrentN, 1, getDeviceName(), "STEPPER_CURRENT", "Stepper current", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	IUFillSwitch(&Switch1S[0], "SW1ON", "ON", ISS_OFF);
	IUFillSwitch(&Switch1S[1], "SW1OFF", "OFF", ISS_ON);
	IUFillSwitchVector(&Switch1SP, Switch1S, 2, getDeviceName(), "SWITCH_1", RelayLabelsT[0].text, OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillSwitch(&Switch2S[0], "SW2ON", "ON", ISS_OFF);
	IUFillSwitch(&Switch2S[1], "SW2OFF", "OFF", ISS_ON);
	IUFillSwitchVector(&Switch2SP, Switch2S, 2, getDeviceName(), "SWITCH_2", RelayLabelsT[1].text, OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillNumber(&PWM1N[0], "PWMout1", "%", "%0.0f", 0, 100, 10, 0);
	IUFillNumberVector(&PWM1NP, PWM1N, 1, getDeviceName(), "PWMOUT1", RelayLabelsT[2].text, OUTPUTS_TAB, IP_RW, 60, IPS_IDLE);

	IUFillNumber(&PWM2N[0], "PWMout2", "%", "%0.0f", 0, 100, 10, 0);
	IUFillNumberVector(&PWM2NP, PWM2N, 1, getDeviceName(), "PWMOUT2", RelayLabelsT[3].text, OUTPUTS_TAB, IP_RW, 60, IPS_IDLE);

	// Power readings
	IUFillNumber(&PowerReadingsN[POW_VIN], "POW_VIN", "Input voltage [V]", "%0.2f", 0, 15, 10, 0);
	IUFillNumber(&PowerReadingsN[POW_VREG], "POW_VREG", "Regulated voltage [V]", "%0.2f", 0, 15, 10, 0);
	IUFillNumber(&PowerReadingsN[POW_ITOT], "POW_ITOT", "Total current [A]", "%0.2f", 0, 20, 1, 0);
	IUFillNumber(&PowerReadingsN[POW_PTOT], "POW_PTOT", "Total power [W]", "%0.1f", 0, 200, 1, 0);
	IUFillNumber(&PowerReadingsN[POW_AH], "POW_AH", "Energy consumed [Ah]", "%0.2f", 0, 10000, 1, 0);
	IUFillNumber(&PowerReadingsN[POW_WH], "POW_WH", "Energy consumed [Wh]", "%0.2f", 0, 100000, 1, 0);
	IUFillNumberVector(&PowerReadingsNP, PowerReadingsN, 6, getDeviceName(), "POWER_READINGS", "Power readings", OUTPUTS_TAB, IP_RO, 60, IPS_IDLE);	

	// Environment Group
	addParameter("WEATHER_TEMPERATURE", "Temperature [C]", -15, 35, 15);
	addParameter("WEATHER_HUMIDITY", "Humidity %", 0, 100, 15);
	addParameter("WEATHER_DEWPOINT", "Dew Point [C]", -25, 20, 15);
	addParameter("WEATHER_SKY_TEMP", "Sky temperature [C]", -50, 20, 20);
	addParameter("WEATHER_SKY_DIFF", "Temperature difference [C]", -5, 40, 10);
	addParameter("SQM_READING", "Sky brightness [mag/arcsec2]", 10, 25, 15);

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
	FocusAbsPosN[0].step = (int)FocusAbsPosN[0].max / 100;

	FocusMotionS[FOCUS_OUTWARD].s = ISS_ON;
	FocusMotionS[FOCUS_INWARD].s = ISS_OFF;
	IDSetSwitch(&FocusMotionSP, nullptr);

	return true;
}

bool AstroLink4Pi::updateProperties()
{
	INDI::DefaultDevice::updateProperties();

	if (isConnected())
	{
		FI::updateProperties();
		WI::updateProperties();

		defineProperty(&ActiveTelescopeTP);
		defineProperty(&FocuserTravelNP);
		defineProperty(&FocusResolutionSP);
		defineProperty(&FocusHoldSP);
		defineProperty(&FocuserInfoNP);
		defineProperty(&FocusStepDelayNP);
		defineProperty(&FocusBacklashNP);
		defineProperty(&SysTimeTP);
		defineProperty(&SysInfoTP);
		defineProperty(&SysControlSP);
		defineProperty(&Switch1SP);
		defineProperty(&Switch2SP);
		defineProperty(&PWM1NP);
		defineProperty(&PWM2NP);
		defineProperty(&PWMcycleNP);
		defineProperty(&StepperCurrentNP);

		IDSnoopDevice(ActiveTelescopeT[0].text, "TELESCOPE_INFO");

		defineProperty(&FocusTemperatureNP);
		defineProperty(&TemperatureCoefNP);
		defineProperty(&TemperatureCompensateSP);
		defineProperty(&PowerReadingsNP);
	}
	else
	{
		deleteProperty(ActiveTelescopeTP.name);
		deleteProperty(FocuserTravelNP.name);
		deleteProperty(FocusResolutionSP.name);
		deleteProperty(FocusHoldSP.name);
		deleteProperty(FocuserInfoNP.name);
		deleteProperty(FocusStepDelayNP.name);
		deleteProperty(FocusBacklashNP.name);
		deleteProperty(FocusTemperatureNP.name);
		deleteProperty(TemperatureCoefNP.name);
		deleteProperty(TemperatureCompensateSP.name);
		deleteProperty(SysTimeTP.name);
		deleteProperty(SysInfoTP.name);
		deleteProperty(SysControlSP.name);
		deleteProperty(Switch1SP.name);
		deleteProperty(Switch2SP.name);
		deleteProperty(PWM1NP.name);
		deleteProperty(PWM2NP.name);
		deleteProperty(PWMcycleNP.name);
		deleteProperty(StepperCurrentNP.name);
		deleteProperty(PowerReadingsNP.name);
		FI::updateProperties();
		WI::updateProperties();
	}

	return true;
}

bool AstroLink4Pi::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
		// handle focus step delay
		if (!strcmp(name, FocusStepDelayNP.name))
		{
			IUUpdateNumber(&FocusStepDelayNP, values, names, n);
			FocusStepDelayNP.s = IPS_BUSY;
			IDSetNumber(&FocusStepDelayNP, nullptr);
			FocusStepDelayNP.s = IPS_OK;
			IDSetNumber(&FocusStepDelayNP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Step delay set to %0.0f us.", FocusStepDelayN[0].value);
			return true;
		}

		// handle focus maximum position
		if (!strcmp(name, FocusMaxPosNP.name))
		{
			IUUpdateNumber(&FocusMaxPosNP, values, names, n);

			FocusAbsPosN[0].max = FocusMaxPosN[0].value;
			IUUpdateMinMax(&FocusAbsPosNP); // This call is not INDI protocol compliant

			FocusAbsPosNP.s = IPS_OK;
			IDSetNumber(&FocusMaxPosNP, nullptr);
			getFocuserInfo();
			return true;
		}

		// handle temperature coefficient
		if (!strcmp(name, TemperatureCoefNP.name))
		{
			IUUpdateNumber(&TemperatureCoefNP, values, names, n);
			TemperatureCoefNP.s = IPS_OK;
			IDSetNumber(&TemperatureCoefNP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Temperature coefficient set to %0.1f steps/°C", TemperatureCoefN[0].value);
			return true;
		}

		// handle focuser travel
		if (!strcmp(name, FocuserTravelNP.name))
		{
			IUUpdateNumber(&FocuserTravelNP, values, names, n);
			getFocuserInfo();
			FocuserTravelNP.s = IPS_OK;
			IDSetNumber(&FocuserTravelNP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Maximum focuser travel set to %0.0f mm", FocuserTravelN[0].value);
			return true;
		}

		// handle PWMouts
		if (!strcmp(name, PWM1NP.name))
		{
			IUUpdateNumber(&PWM1NP, values, names, n);
			PWM1NP.s = IPS_OK;
			IDSetNumber(&PWM1NP, nullptr);
			lgTxPwm(pigpioHandle, PWM1_PIN, PWMcycleN[0].value, PWM1N[0].value, 0, 0);
			// set_PWM_dutycycle(pigpioHandle, PWM1_PIN, PWM1N[0].value);
			pwmState[0] = PWM1N[0].value;
			DEBUGF(INDI::Logger::DBG_SESSION, "PWM 1 set to %0.0f", PWM1N[0].value);
			return true;
		}

		if (!strcmp(name, PWM2NP.name))
		{
			IUUpdateNumber(&PWM2NP, values, names, n);
			PWM2NP.s = IPS_OK;
			IDSetNumber(&PWM2NP, nullptr);
			lgTxPwm(pigpioHandle, PWM2_PIN, PWMcycleN[0].value, PWM1N[0].value, 0, 0);
			// set_PWM_dutycycle(pigpioHandle, PWM2_PIN, PWM2N[0].value);
			pwmState[1] = PWM2N[0].value;
			DEBUGF(INDI::Logger::DBG_SESSION, "PWM 2 set to %0.0f", PWM2N[0].value);
			return true;
		}

		// handle PWMcycle
		if (!strcmp(name, PWMcycleNP.name))
		{
			IUUpdateNumber(&PWMcycleNP, values, names, n);
			PWMcycleNP.s = IPS_OK;
			IDSetNumber(&PWMcycleNP, nullptr);
			lgTxPwm(pigpioHandle, PWM1_PIN, PWMcycleN[0].value, PWM1N[0].value, 0, 0);
			lgTxPwm(pigpioHandle, PWM2_PIN, PWMcycleN[0].value, PWM1N[0].value, 0, 0);
			// set_PWM_frequency(pigpioHandle, PWM1_PIN, PWMcycleN[0].value);
			// set_PWM_frequency(pigpioHandle, PWM2_PIN, PWMcycleN[0].value);
			DEBUGF(INDI::Logger::DBG_SESSION, "PWM frequency set to %0.0f Hz", PWMcycleN[0].value);
			return true;
		}

		// handle stepper current
		if (!strcmp(name, StepperCurrentNP.name))
		{
			IUUpdateNumber(&StepperCurrentNP, values, names, n);
			StepperCurrentNP.s = IPS_OK;
			IDSetNumber(&StepperCurrentNP, nullptr);
			stepperCurrent = StepperCurrentN[0].value;
			DEBUGF(INDI::Logger::DBG_SESSION, "Stepper current set to %0.0f mA", StepperCurrentN[0].value);
			setCurrent(true);
			return true;
		}

		if (strstr(name, "FOCUS_"))
			return FI::processNumber(dev, name, values, names, n);
		if (strstr(name, "WEATHER_"))
			return WI::processNumber(dev, name, values, names, n);
	}

	return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool AstroLink4Pi::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	int rv;

	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
		// handle temperature compensation
		if (!strcmp(name, TemperatureCompensateSP.name))
		{
			IUUpdateSwitch(&TemperatureCompensateSP, states, names, n);

			if (TemperatureCompensateS[0].s == ISS_ON)
			{
				TemperatureCompensateSP.s = IPS_OK;
				DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation ENABLED.");
			}

			if (TemperatureCompensateS[1].s == ISS_ON)
			{
				TemperatureCompensateSP.s = IPS_IDLE;
				DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation DISABLED.");
			}

			IDSetSwitch(&TemperatureCompensateSP, nullptr);
			return true;
		}

		// handle system control
		if (!strcmp(name, SysControlSP.name))
		{
			IUUpdateSwitch(&SysControlSP, states, names, n);

			if (SysControlS[0].s == ISS_ON)
			{
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink device is set to REBOOT. Confirm or Cancel operation.");
				SysControlSP.s = IPS_BUSY;
				IDSetSwitch(&SysControlSP, NULL);

				// confirm switch
				defineProperty(&SysOpConfirmSP);

				return true;
			}
			if (SysControlS[1].s == ISS_ON)
			{
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink device is set to SHUT DOWN. Confirm or Cancel operation.");
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

			if (SysOpConfirmS[0].s == ISS_ON)
			{
				SysOpConfirmSP.s = IPS_IDLE;
				IDSetSwitch(&SysOpConfirmSP, NULL);
				SysOpConfirmS[0].s = ISS_OFF;
				IDSetSwitch(&SysOpConfirmSP, NULL);

				// execute system operation
				if (SysControlS[0].s == ISS_ON)
				{
					DEBUG(INDI::Logger::DBG_SESSION, "System operation confirmed. System is going to REBOOT now");
					FILE *pipe;
					char buffer[512];
					pipe = popen("sudo reboot", "r");
					if (fgets(buffer, 512, pipe) == NULL)
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
					FILE *pipe;
					char buffer[512];
					pipe = popen("sudo poweroff", "r");
					if (fgets(buffer, 512, pipe) == NULL)
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

			if (SysOpConfirmS[1].s == ISS_ON)
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

		// handle relay 1
		if (!strcmp(name, Switch1SP.name))
		{
			IUUpdateSwitch(&Switch1SP, states, names, n);

			if (Switch1S[0].s == ISS_ON)
			{
				rv = lgGpioWrite(pigpioHandle, OUT1_PIN, 1);
				if (rv != 0)
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting AstroLink Relay #1");
					Switch1SP.s = IPS_ALERT;
					Switch1S[0].s = ISS_OFF;
					IDSetSwitch(&Switch1SP, NULL);
					return false;
				}
				relayState[0] = 1;
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink Relays #1 set to ON");
				Switch1SP.s = IPS_OK;
				Switch1S[1].s = ISS_OFF;
				IDSetSwitch(&Switch1SP, NULL);
				return true;
			}
			if (Switch1S[1].s == ISS_ON)
			{
				rv = lgGpioWrite(pigpioHandle, OUT1_PIN, 0);
				if (rv != 0)
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting AstroLink Relay #1");
					Switch1SP.s = IPS_ALERT;
					Switch1S[1].s = ISS_OFF;
					IDSetSwitch(&Switch1SP, NULL);
					return false;
				}
				relayState[0] = 0;
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink Relays #1 set to OFF");
				Switch1SP.s = IPS_IDLE;
				Switch1S[0].s = ISS_OFF;
				IDSetSwitch(&Switch1SP, NULL);
				return true;
			}
		}

		// handle relay 2
		if (!strcmp(name, Switch2SP.name))
		{
			IUUpdateSwitch(&Switch2SP, states, names, n);

			if (Switch2S[0].s == ISS_ON)
			{
				rv = lgGpioWrite(pigpioHandle, OUT2_PIN, 1);
				if (rv != 0)
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting AstroLink Relay #2");
					Switch2SP.s = IPS_ALERT;
					Switch2S[0].s = ISS_OFF;
					IDSetSwitch(&Switch2SP, NULL);
					return false;
				}
				relayState[1] = 1;
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink Relays #2 set to ON");
				Switch2SP.s = IPS_OK;
				Switch2S[1].s = ISS_OFF;
				IDSetSwitch(&Switch2SP, NULL);
				return true;
			}
			if (Switch2S[1].s == ISS_ON)
			{
				rv = lgGpioWrite(pigpioHandle, OUT2_PIN, 0);
				if (rv != 0)
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting AstroLink Relay #2");
					Switch2SP.s = IPS_ALERT;
					Switch2S[1].s = ISS_OFF;
					IDSetSwitch(&Switch2SP, NULL);
					return false;
				}
				relayState[1] = 0;
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink Relays #2 set to OFF");
				Switch2SP.s = IPS_IDLE;
				Switch2S[0].s = ISS_OFF;
				IDSetSwitch(&Switch2SP, NULL);
				return true;
			}
		}

		// handle focus motor hold
		if (!strcmp(name, FocusHoldSP.name))
		{
			IUUpdateSwitch(&FocusHoldSP, states, names, n);

			if (FocusHoldS[0].s == ISS_ON)
				holdPower = 0;

			if (FocusHoldS[1].s == ISS_ON)
				holdPower = 1;

			if (FocusHoldS[2].s == ISS_ON)
				holdPower = 2;

			if (FocusHoldS[3].s == ISS_ON)
				holdPower = 3;

			if (FocusHoldS[4].s == ISS_ON)
				holdPower = 4;

			if (FocusHoldS[5].s == ISS_ON)
				holdPower = 5;

			FocusHoldSP.s = IPS_OK;
			IDSetSwitch(&FocusHoldSP, nullptr);
			setCurrent(true);
			return true;
		}

		// handle focus resolution
		if (!strcmp(name, FocusResolutionSP.name))
		{
			int last_resolution = resolution;

			IUUpdateSwitch(&FocusResolutionSP, states, names, n);

			// Resolution 1/1
			if (FocusResolutionS[0].s == ISS_ON)
				resolution = 1;

			// Resolution 1/2
			if (FocusResolutionS[1].s == ISS_ON)
				resolution = 2;

			// Resolution 1/4
			if (FocusResolutionS[2].s == ISS_ON)
				resolution = 4;

			// Resolution 1/8
			if (FocusResolutionS[3].s == ISS_ON)
				resolution = 8;

			// Resolution 1/16
			if (FocusResolutionS[4].s == ISS_ON)
				resolution = 16;

			// Resolution 1/32
			if (FocusResolutionS[5].s == ISS_ON)
				resolution = 32;

			// Adjust position to a step in lower resolution
			int position_adjustment = last_resolution * (FocusAbsPosN[0].value / last_resolution - (int)FocusAbsPosN[0].value / last_resolution);
			if (resolution < last_resolution && position_adjustment > 0)
			{
				if ((float)position_adjustment / last_resolution < 0.5)
				{
					position_adjustment *= -1;
				}
				else
				{
					position_adjustment = last_resolution - position_adjustment;
				}
				DEBUGF(INDI::Logger::DBG_SESSION, "Focuser position adjusted by %d steps at 1/%d resolution to sync with 1/%d resolution.", position_adjustment, last_resolution, resolution);
				MoveAbsFocuser(FocusAbsPosN[0].value + position_adjustment);
			}

			SetResolution(resolution);

			// update values based on resolution
			FocusRelPosN[0].min = (int)FocusRelPosN[0].min * resolution / last_resolution;
			FocusRelPosN[0].max = (int)FocusRelPosN[0].max * resolution / last_resolution;
			FocusRelPosN[0].step = (int)FocusRelPosN[0].step * resolution / last_resolution;
			FocusRelPosN[0].value = (int)FocusRelPosN[0].value * resolution / last_resolution;
			IDSetNumber(&FocusRelPosNP, nullptr);
			IUUpdateMinMax(&FocusRelPosNP);

			FocusAbsPosN[0].max = (int)FocusAbsPosN[0].max * resolution / last_resolution;
			FocusAbsPosN[0].step = (int)FocusAbsPosN[0].step * resolution / last_resolution;
			FocusAbsPosN[0].value = (int)FocusAbsPosN[0].value * resolution / last_resolution;
			IDSetNumber(&FocusAbsPosNP, nullptr);
			IUUpdateMinMax(&FocusAbsPosNP);

			FocusMaxPosN[0].min = (int)FocusMaxPosN[0].min * resolution / last_resolution;
			FocusMaxPosN[0].max = (int)FocusMaxPosN[0].max * resolution / last_resolution;
			FocusMaxPosN[0].step = (int)FocusMaxPosN[0].step * resolution / last_resolution;
			FocusMaxPosN[0].value = (int)FocusMaxPosN[0].value * resolution / last_resolution;
			IDSetNumber(&FocusMaxPosNP, nullptr);
			IUUpdateMinMax(&FocusMaxPosNP);

			getFocuserInfo();

			FocusResolutionSP.s = IPS_OK;
			IDSetSwitch(&FocusResolutionSP, nullptr);
			return true;
		}

		if (strstr(name, "FOCUS"))
			return FI::processSwitch(dev, name, states, names, n);
	}

	return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool AstroLink4Pi::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
		// handle active devices
		if (!strcmp(name, ActiveTelescopeTP.name))
		{
			IUUpdateText(&ActiveTelescopeTP, texts, names, n);

			IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, ActiveTelescopeT[0].text, "TELESCOPE_INFO", "Scope Properties", OPTIONS_TAB, IP_RW, 60, IPS_OK);
			IDSnoopDevice(ActiveTelescopeT[0].text, "TELESCOPE_INFO");

			ActiveTelescopeTP.s = IPS_OK;
			IDSetText(&ActiveTelescopeTP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Active telescope set to %s.", ActiveTelescopeT[0].text);
			return true;
		}

		// handle relay labels
		if (!strcmp(name, RelayLabelsTP.name))
		{
			if (isConnected())
			{
				DEBUG(INDI::Logger::DBG_WARNING, "Cannot set labels while device is connected.");
				return false;
			}

			IUUpdateText(&RelayLabelsTP, texts, names, n);
			RelayLabelsTP.s = IPS_OK;
			IDSetText(&RelayLabelsTP, nullptr);
			DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi labels set . You need to save configuration and restart driver to activate the changes.");
			DEBUGF(INDI::Logger::DBG_DEBUG, "AstroLink 4 Pi labels set to OUT1: %s, OUT2: %s, PWM1: %s, PWM2: %s", RelayLabelsT[0].text, RelayLabelsT[1].text, RelayLabelsT[2].text, RelayLabelsT[3].text);

			return true;
		}
	}

	return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool AstroLink4Pi::ISSnoopDevice(XMLEle *root)
{
	if (IUSnoopNumber(root, &ScopeParametersNP) == 0)
	{
		getFocuserInfo();
		DEBUGF(INDI::Logger::DBG_DEBUG, "Telescope parameters: %0.0f, %0.0f.", ScopeParametersN[0].value, ScopeParametersN[1].value);
		return true;
	}

	return INDI::DefaultDevice::ISSnoopDevice(root);
}

bool AstroLink4Pi::saveConfigItems(FILE *fp)
{
	FI::saveConfigItems(fp);
	WI::saveConfigItems(fp);
	IUSaveConfigText(fp, &ActiveTelescopeTP);
	IUSaveConfigSwitch(fp, &FocusResolutionSP);
	IUSaveConfigSwitch(fp, &FocusHoldSP);
	IUSaveConfigSwitch(fp, &FocusReverseSP);
	IUSaveConfigSwitch(fp, &TemperatureCompensateSP);
	IUSaveConfigNumber(fp, &FocusMaxPosNP);
	IUSaveConfigNumber(fp, &FocusStepDelayNP);
	IUSaveConfigNumber(fp, &FocusBacklashNP);
	IUSaveConfigNumber(fp, &FocuserTravelNP);
	IUSaveConfigNumber(fp, &TemperatureCoefNP);
	IUSaveConfigNumber(fp, &PWMcycleNP);
	IUSaveConfigText(fp, &RelayLabelsTP);
	IUSaveConfigSwitch(fp, &Switch1SP);
	IUSaveConfigSwitch(fp, &Switch2SP);
	IUSaveConfigNumber(fp, &StepperCurrentNP);
	IUSaveConfigNumber(fp, &PWM1NP);
	IUSaveConfigNumber(fp, &PWM2NP);

	return true;
}

void AstroLink4Pi::TimerHit()
{
	if (!isConnected())
		return;

	long int timeMillis = millis();

	if (nextTemperatureRead < timeMillis)
	{
		SHTavailable = readSHT();
		MLXavailable = readMLX();
		SQMavailable = readSQM();

		nextTemperatureRead = timeMillis + TEMPERATURE_UPDATE_TIMEOUT;

		if (DSavailable || SHTavailable || MLXavailable)
		{
			FocusTemperatureN[0].value = focuserTemperature;
			FocusTemperatureNP.s = IPS_OK;
			ParametersNP.s = IPS_OK;
		}
		else
		{
			FocusTemperatureN[0].value = 0.0;
			FocusTemperatureNP.s = IPS_ALERT;
			ParametersNP.s = IPS_ALERT;
			IDSetNumber(&FocusTemperatureNP, nullptr);
		}
		IDSetNumber(&ParametersNP, nullptr);
		IDSetNumber(&FocusTemperatureNP, nullptr);
	}
	if (nextTemperatureCompensation < timeMillis)
	{
		temperatureCompensation();
		nextTemperatureCompensation = timeMillis + TEMPERATURE_COMPENSATION_TIMEOUT;
	}
	if (nextSystemRead < timeMillis)
	{
		systemUpdate();
		nextSystemRead = timeMillis + SYSTEM_UPDATE_PERIOD;
	}
	if (nextFanUpdate < timeMillis)
	{
		fanUpdate();
		nextFanUpdate = timeMillis + FAN_PERIOD;
	}
	readPower();

	SetTimer(POLL_PERIOD);
}

bool AstroLink4Pi::AbortFocuser()
{
	if (_motionThread.joinable())
	{
		_abort = true;
		_motionThread.join();
	}
	DEBUG(INDI::Logger::DBG_SESSION, "Focuser motion aborted.");
	return true;
}

IPState AstroLink4Pi::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
	uint32_t targetTicks = FocusAbsPosN[0].value + ((int32_t)ticks * (dir == FOCUS_INWARD ? -1 : 1));
	return MoveAbsFocuser(targetTicks);
}

IPState AstroLink4Pi::MoveAbsFocuser(uint32_t targetTicks)
{
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
	setCurrent(false);

	// set direction
	const char *direction;
	int newDirection;
	if (targetTicks > FocusAbsPosN[0].value)
	{
		// OUTWARD
		direction = "OUTWARD";
		newDirection = 1;
	}
	else
	{
		// INWARD
		direction = "INWARD";
		newDirection = -1;
	}

	// if direction changed do backlash adjustment
	if (lastDirection != 0 && newDirection != lastDirection && FocusBacklashN[0].value != 0)
	{
		DEBUGF(INDI::Logger::DBG_SESSION, "Backlash compensation by %0.0f steps.", FocusBacklashN[0].value);
		backlashTicksRemaining = FocusBacklashN[0].value;
	}
	else
	{
		backlashTicksRemaining = 0;
	}

	lastDirection = newDirection;

	DEBUGF(INDI::Logger::DBG_SESSION, "Focuser is moving %s to position %d.", direction, targetTicks);

	if (_motionThread.joinable())
	{
		_abort = true;
		_motionThread.join();
	}

	_abort = false;
	_motionThread = std::thread([this](uint32_t targetPos, int direction, int pigpioHandle, int backlashTicksRemaining)
								{
									int motorDirection = direction;

									uint32_t currentPos = FocusAbsPosN[0].value;
									while (currentPos != targetPos && !_abort)
									{
										if (currentPos % 100 == 0)
										{
											FocusAbsPosN[0].value = currentPos;
											FocusAbsPosNP.s = IPS_BUSY;
											IDSetNumber(&FocusAbsPosNP, nullptr);
										}
										if (FocusReverseS[INDI_ENABLED].s == ISS_ON)
										{
											lgGpioWrite(pigpioHandle, DIR_PIN, (motorDirection < 0) ? 1 : 0);
										}
										else
										{
											lgGpioWrite(pigpioHandle, DIR_PIN, (motorDirection < 0) ? 0 : 1);
										}
										lgGpioWrite(pigpioHandle, STP_PIN, 1);
										usleep(10);
										lgGpioWrite(pigpioHandle, STP_PIN, 0);

										if (backlashTicksRemaining <= 0)
										{ // Only Count the position change if it is not due to backlash
											currentPos += motorDirection;
										}
										else
										{ // Don't count the backlash position change, just decrement the counter
											backlashTicksRemaining -= 1;
										}
										usleep(FocusStepDelayN[0].value);
										// auto start = std::chrono::high_resolution_clock::now();
										// for (;;)
										// {
										// 	auto later = std::chrono::high_resolution_clock::now();
										// 	auto micros = std::chrono::duration_cast<std::chrono::microseconds>(later - start);
										// 	if (micros.count() >= (int)FocusStepDelayN[0].value)
										// 		break;
										// }
										// std::this_thread::sleep_for(std::chrono::microseconds((int) FocusStepDelayN[0].value));
									}

									// update abspos value and status
									DEBUGF(INDI::Logger::DBG_SESSION, "Focuser moved to position %i", (int)currentPos);
									FocusAbsPosN[0].value = currentPos;
									FocusAbsPosNP.s = IPS_OK;
									IDSetNumber(&FocusAbsPosNP, nullptr);
									FocusRelPosNP.s = IPS_OK;
									IDSetNumber(&FocusRelPosNP, nullptr);

									savePosition((int)FocusAbsPosN[0].value * MAX_RESOLUTION / resolution); // always save at MAX_RESOLUTION
									lastTemperature = FocusTemperatureN[0].value;							// register last temperature
									setCurrent(true); },
								targetTicks, lastDirection, pigpioHandle, backlashTicksRemaining);

	return IPS_BUSY;
}

void AstroLink4Pi::SetResolution(int res)
{
	// Release lines
	lgGpioWrite(pigpioHandle, M0_PIN, 1);
	lgGpioWrite(pigpioHandle, M1_PIN, 1);
	lgGpioWrite(pigpioHandle, M2_PIN, 1);

	switch (res)
	{
	case 1: // 1:1

		lgGpioWrite(pigpioHandle, M0_PIN, 0);
		lgGpioWrite(pigpioHandle, M1_PIN, 0);
		lgGpioWrite(pigpioHandle, M2_PIN, 0);
		break;
	case 2: // 1:2
		lgGpioWrite(pigpioHandle, M0_PIN, 1);
		lgGpioWrite(pigpioHandle, M1_PIN, 0);
		lgGpioWrite(pigpioHandle, M2_PIN, 0);
		break;
	case 4: // 1:4
		lgGpioWrite(pigpioHandle, M0_PIN, 0);
		lgGpioWrite(pigpioHandle, M1_PIN, 1);
		lgGpioWrite(pigpioHandle, M2_PIN, 0);
		break;
	case 8: // 1:8
		lgGpioWrite(pigpioHandle, M0_PIN, 1);
		lgGpioWrite(pigpioHandle, M1_PIN, 1);
		lgGpioWrite(pigpioHandle, M2_PIN, 0);
		break;
	case 16: // 1:16
		lgGpioWrite(pigpioHandle, M0_PIN, 0);
		lgGpioWrite(pigpioHandle, M1_PIN, 0);
		lgGpioWrite(pigpioHandle, M2_PIN, 1);
		break;
	case 32: // 1:32
		lgGpioWrite(pigpioHandle, M0_PIN, 1);
		lgGpioWrite(pigpioHandle, M1_PIN, 1);
		lgGpioWrite(pigpioHandle, M2_PIN, 1);
		break;
	default: // 1:1
		lgGpioWrite(pigpioHandle, M0_PIN, 0);
		lgGpioWrite(pigpioHandle, M1_PIN, 0);
		lgGpioWrite(pigpioHandle, M2_PIN, 0);

		break;
	}

	DEBUGF(INDI::Logger::DBG_SESSION, "Resolution set to 1 / %d.", res);
}

bool AstroLink4Pi::ReverseFocuser(bool enabled)
{
	if (enabled)
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Reverse direction ENABLED.");
	}
	else
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Reverse direction DISABLED.");
	}
	return true;
}

int AstroLink4Pi::savePosition(int pos)
{
	FILE *pFile;
	char posFileName[MAXRBUF];
	char buf[100];

	if (getenv("INDICONFIG"))
	{
		snprintf(posFileName, MAXRBUF, "%s.position", getenv("INDICONFIG"));
	}
	else
	{
		snprintf(posFileName, MAXRBUF, "%s/.indi/%s.position", getenv("HOME"), getDeviceName());
	}

	if (pos == -1)
	{
		pFile = fopen(posFileName, "r");
		if (pFile == NULL)
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "Failed to open file %s.", posFileName);
			return -1;
		}

		if (fgets(buf, 100, pFile) == NULL)
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "Failed to read file %s.", posFileName);
			return -1;
		}
		else
		{
			pos = atoi(buf);
			DEBUGF(INDI::Logger::DBG_DEBUG, "Reading position %d from %s.", pos, posFileName);
		}
	}
	else
	{
		pFile = fopen(posFileName, "w");
		if (pFile == NULL)
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "Failed to open file %s.", posFileName);
			return -1;
		}

		sprintf(buf, "%d", pos);
		fputs(buf, pFile);
		DEBUGF(INDI::Logger::DBG_DEBUG, "Writing position %s to %s.", buf, posFileName);
	}

	fclose(pFile);

	return pos;
}

bool AstroLink4Pi::SyncFocuser(uint32_t ticks)
{
	FocusAbsPosN[0].value = ticks;
	IDSetNumber(&FocusAbsPosNP, nullptr);
	savePosition(ticks);

	DEBUGF(INDI::Logger::DBG_SESSION, "Absolute Position reset to %0.0f", FocusAbsPosN[0].value);

	return true;
}

bool AstroLink4Pi::SetFocuserBacklash(int32_t steps)
{
	DEBUGF(INDI::Logger::DBG_SESSION, "Backlash set to %i steps", steps);
	return true;
}

bool AstroLink4Pi::SetFocuserMaxPosition(uint32_t ticks)
{
	DEBUGF(INDI::Logger::DBG_SESSION, "Max position set to %i steps", ticks);
	return true;
}

void AstroLink4Pi::temperatureCompensation()
{
	if (!isConnected())
		return;

	if (TemperatureCompensateS[0].s == ISS_ON && FocusTemperatureN[0].value != lastTemperature)
	{
		float deltaTemperature = FocusTemperatureN[0].value - lastTemperature; // change of temperature from last focuser movement
		float deltaPos = TemperatureCoefN[0].value * deltaTemperature;

		// Move focuser once the compensation is larger than 1/2 CFZ
		if (abs(deltaPos) > (FocuserInfoN[2].value / 2))
		{
			int thermalAdjustment = round(deltaPos);				   // adjust focuser by half number of steps to keep it in the center of cfz
			MoveAbsFocuser(FocusAbsPosN[0].value + thermalAdjustment); // adjust focuser position
			lastTemperature = FocusTemperatureN[0].value;			   // register last temperature
			DEBUGF(INDI::Logger::DBG_SESSION, "Focuser adjusted by %d steps due to temperature change by %0.2f°C", thermalAdjustment, deltaTemperature);
		}
	}
}

void AstroLink4Pi::setCurrent(bool standby)
{
	if (!isConnected())
		return;

	if (standby)
	{
		lgGpioWrite(pigpioHandle, EN_PIN,  (holdPower > 0) ? 0 : 1);
		lgGpioWrite(pigpioHandle, DECAY_PIN, 0);
		lgTxPwm(pigpioHandle, MOTOR_PWM, 5000, getMotorPWM(holdPower * stepperCurrent / 5), 0, 0);
		if (holdPower > 0)
		{
			DEBUGF(INDI::Logger::DBG_SESSION, "Stepper motor enabled %d %%.", holdPower * 20);
		}
		else
		{
			DEBUG(INDI::Logger::DBG_SESSION, "Stepper motor disabled.");
		}			
	}
	else
	{
		lgGpioWrite(pigpioHandle, EN_PIN, 0);
		lgGpioWrite(pigpioHandle, DECAY_PIN, 1);
		lgTxPwm(pigpioHandle, MOTOR_PWM, 5000, getMotorPWM(stepperCurrent), 0, 0);
	}
}

void AstroLink4Pi::systemUpdate()
{
	// update time
	struct tm *local_timeinfo;
	static char ts[32];
	time_t rawtime;
	time(&rawtime);
	local_timeinfo = localtime(&rawtime);
	strftime(ts, 20, "%Y-%m-%dT%H:%M:%S", local_timeinfo);
	IUSaveText(&SysTimeT[0], ts);
	snprintf(ts, sizeof(ts), "%4.2f", (local_timeinfo->tm_gmtoff / 3600.0));
	IUSaveText(&SysTimeT[1], ts);
	SysTimeTP.s = IPS_OK;
	IDSetText(&SysTimeTP, NULL);

	SysInfoTP.s = IPS_BUSY;
	IDSetText(&SysInfoTP, NULL);

	FILE *pipe;
	char buffer[128];

	// update CPU temp
	pipe = popen("echo $(($(cat /sys/class/thermal/thermal_zone0/temp)/1000))", "r");
	if (fgets(buffer, 128, pipe) != NULL)
		IUSaveText(&SysInfoT[1], buffer);
	pclose(pipe);

	// update uptime
	pipe = popen("uptime|awk -F, '{print $1}'|awk -Fup '{print $2}'|xargs", "r");
	if (fgets(buffer, 128, pipe) != NULL)
		IUSaveText(&SysInfoT[2], buffer);
	pclose(pipe);

	// update load
	pipe = popen("uptime|awk -F, '{print $3\" /\"$4\" /\"$5}'|awk -F: '{print $2}'|xargs", "r");
	if (fgets(buffer, 128, pipe) != NULL)
		IUSaveText(&SysInfoT[3], buffer);
	pclose(pipe);

	SysInfoTP.s = IPS_OK;
	IDSetText(&SysInfoTP, NULL);
}

void AstroLink4Pi::getFocuserInfo()
{
	// https://www.innovationsforesight.com/education/how-much-focus-error-is-too-much/
	float travel_mm = (float)FocuserTravelN[0].value;
	float aperture = (float)ScopeParametersN[0].value;
	float focal = (float)ScopeParametersN[1].value;
	float f_ratio;

	// handle no snooping data from telescope
	if (aperture * focal != 0)
	{
		f_ratio = focal / aperture;
	}
	else
	{
		f_ratio = 0;
		DEBUG(INDI::Logger::DBG_DEBUG, "No telescope focal length and/or aperture info available.");
	}

	float cfz = 4.88 * 0.520 * pow(f_ratio, 2); // CFZ = 4.88 · λ · f^2
	float step_size = 1000.0 * travel_mm / FocusMaxPosN[0].value;
	float steps_per_cfz = (int)cfz / step_size;

	if (steps_per_cfz >= 4)
	{
		FocuserInfoNP.s = IPS_OK;
	}
	else if (steps_per_cfz > 2 && steps_per_cfz < 4)
	{
		FocuserInfoNP.s = IPS_BUSY;
	}
	else
	{
		FocuserInfoNP.s = IPS_ALERT;
	}

	FocuserInfoN[0].value = step_size;
	FocuserInfoN[1].value = cfz;
	FocuserInfoN[2].value = steps_per_cfz;
	IDSetNumber(&FocuserInfoNP, nullptr);

	DEBUGF(INDI::Logger::DBG_DEBUG, "Focuser Info: %0.2f %0.2f %0.2f.", FocuserInfoN[0].value, FocuserInfoN[1].value, FocuserInfoN[2].value);
}

long int AstroLink4Pi::millis()
{
	// static uint64_t nsec_zero = lguTimestamp();
	// return (int) (lguTimestamp() - nsec_zero) / 1000000;
	struct timespec clock;
	if (clock_gettime(CLOCK_MONOTONIC, &clock) == 0)
	{
		static long int tv_sec_zero = clock.tv_sec;
		int ret = 1000 * (clock.tv_sec - tv_sec_zero) + clock.tv_nsec / 1000000;
		return ret;
	}
	else
	{
		DEBUG(INDI::Logger::DBG_ERROR, "CLOCK_MONOTONIC not available.");
		return 0;
	}
}

int AstroLink4Pi::getMotorPWM(int current)
{
	// 100 = 1.03V = 2.06A, 1 = 20mA
	return current / 20;
}

void AstroLink4Pi::fanUpdate()
{
	int fanPinAvailable = lgGpioClaimOutput(pigpioHandle, 0, FAN_PIN, 0);
	if(fanPinAvailable == 0)
	{
		DEBUGF(INDI::Logger::DBG_SESSION, "GPIO fan set to %s\n", SysInfoT[1].text);
	}
	else
	{
		DEBUGF(INDI::Logger::DBG_SESSION, "GPIO fan pin not available %d\n", fanPinAvailable);
	}
}

bool AstroLink4Pi::readSQM()
{
	char i2cData[7];

	int i2cHandle = lgI2cOpen(1, 0x33, 0);
	if (i2cHandle >= 0)
	{
		int read = lgI2cReadDevice(i2cHandle, i2cData, 7);
		lgI2cClose(i2cHandle);
		if (read > 6)
		{
			int sqm = i2cData[5] * 256 + i2cData[6];
			setParameterValue("SQM_READING", 0.01 * sqm);
			//DEBUGF(INDI::Logger::DBG_SESSION, "SQM read %i %i", i2cData[5], i2cData[6]);
			SQMavailable = true;
		}
		else
		{
			SQMavailable = false;
		}
	}
	else
	{
		SQMavailable = false;
	}
	return SQMavailable;
}

bool AstroLink4Pi::readMLX()
{
	int i2cHandle = lgI2cOpen(1, 0x5A, 0);
	if (i2cHandle >= 0)
	{
		int Tamb = lgI2cReadWordData(i2cHandle, 0x06); 
		int Tobj = lgI2cReadWordData(i2cHandle, 0x07);
		lgI2cClose(i2cHandle);
		if (Tamb >= 0 && Tobj >= 0)
		{
			setParameterValue("WEATHER_SKY_TEMP", 0.02 * Tobj - 273.15);
			setParameterValue("WEATHER_SKY_DIFF", 0.02 * (Tobj - Tamb));
			if (!DSavailable && !SHTavailable)
				focuserTemperature = 0.02 * Tamb - 273.15;
			MLXavailable = true;
		}
		else
		{
			DEBUG(INDI::Logger::DBG_DEBUG, "Cannot read data from MLX sensor.");
			MLXavailable = false;
		}
	}
	else
	{
		DEBUG(INDI::Logger::DBG_DEBUG, "No MLX sensor found.");
		MLXavailable = false;
	}

	if (!MLXavailable)
	{
		setParameterValue("WEATHER_SKY_TEMP", 0.0);
		setParameterValue("WEATHER_SKY_DIFF", 0.0);
	}

	return MLXavailable;
}

bool AstroLink4Pi::readSHT()
{
	char i2cData[6];
	char i2cWrite[2];

	int i2cHandle = lgI2cOpen(1, 0x44, 0);
	if (i2cHandle >= 0)
	{
		i2cWrite[0] = 0x24;
		i2cWrite[1] = 0x00;
		int written = lgI2cWriteDevice(i2cHandle, i2cWrite, 2);
		if (written == 0)
		{
			usleep(30000);
			int read = lgI2cReadDevice(i2cHandle, i2cData, 6);

			if (read > 4)
			{
				int temp = i2cData[0] * 256 + i2cData[1];
				double cTemp = -45.0 + (175.0 * temp / 65535.0);
				double humidity = 100.0 * (i2cData[3] * 256.0 + i2cData[4]) / 65535.0;

				double a = 17.271;
				double b = 237.7;
				double tempAux = (a * cTemp) / (b + cTemp) + log(humidity * 0.01);
				double Td = (b * tempAux) / (a - tempAux);

				setParameterValue("WEATHER_TEMPERATURE", cTemp);
				setParameterValue("WEATHER_HUMIDITY", humidity);
				setParameterValue("WEATHER_DEWPOINT", Td);
				focuserTemperature = cTemp;
				SHTavailable = true;
			}
		}
		else
		{
			DEBUG(INDI::Logger::DBG_DEBUG, "Cannot write data to SHT sensor");
			SHTavailable = false;
		}
		lgI2cClose(i2cHandle);
	}
	else
	{
		DEBUG(INDI::Logger::DBG_DEBUG, "No SHT sensor found.");
		SHTavailable = false;
	}

	if (!SHTavailable)
	{
		setParameterValue("WEATHER_TEMPERATURE", 0.0);
		setParameterValue("WEATHER_HUMIDITY", 0.0);
		setParameterValue("WEATHER_DEWPOINT", 0.0);
	}
	return SHTavailable;
}

bool AstroLink4Pi::readPower() 
{
	if(revision < 4) return false;

	char writeBuf[3];
	char readBuf[2];

	int i2cHandle = lgI2cOpen(1, 0x48, 0);
	if (i2cHandle >= 0)
	{
		/*
		powerIndex 0-1 Vin WR, 2-3 Vreg WR, 4-5 Itot WR

		15 		- 1 	start single conv
		14:12	- 100 	Vin, 101 Vreg, 110 Itot, 111 Iref, 011 Ireal
		11:9  	- 001	+-4.096V
		8		- 1 single

		7:5		- 010 32SPS, 011 64SPS, 001 16SPS
		4:2		- 000 comparator
		1:0		- 11 comparator disable
		*/

		writeBuf[0] = 0x01;
		writeBuf[1] = 0b11000011;
		writeBuf[2] = 0b00100011;
		if((powerIndex % 2) == 0)	// Trigger conversion
		{
			switch(powerIndex) 
			{
				case 0: writeBuf[1] = 0b11000011; break;
				case 2: writeBuf[1] = 0b11010011; break;
				case 4: writeBuf[1] = 0b10110011; break;
			}
			int written = lgI2cWriteDevice(i2cHandle, writeBuf, 3);
			if(written != 0)
			{
				DEBUG(INDI::Logger::DBG_DEBUG, "Cannot write data to power sensor");
				PowerReadingsNP.s = IPS_ALERT;
			}
		}
		else						// Trigger read
		{
			PowerReadingsNP.s = IPS_BUSY;

			writeBuf[0] = 0x00;
			int written = lgI2cWriteDevice(i2cHandle, writeBuf, 1);
			if(written == 0)
			{
				int read = lgI2cReadDevice(i2cHandle, readBuf, 2);
				if(read > 0)
				{
					int16_t val = readBuf[0] * 255 + readBuf[1];

					switch(powerIndex)
					{
						case 1: PowerReadingsN[POW_VIN].value = (float) val / 32768.0 * 4.096 * 6.6; break;
						case 3: PowerReadingsN[POW_VREG].value = (float) val / 32768.0 * 4.096 * 6.6; break;
						case 5: PowerReadingsN[POW_ITOT].value = (float) val / 32768.0 * 4.096 * 1 * 10; break;
					}
					PowerReadingsN[POW_PTOT].value = PowerReadingsN[POW_VIN].value * PowerReadingsN[POW_ITOT].value;
					energyAs += PowerReadingsN[POW_ITOT].value * 0.4;
					energyWs += PowerReadingsN[POW_VIN].value * PowerReadingsN[POW_ITOT].value * 0.4;
					PowerReadingsN[POW_AH].value = energyAs / 3600;
					PowerReadingsN[POW_WH].value = energyWs / 3600;

					PowerReadingsNP.s = IPS_OK;
				}
				else
				{
					DEBUG(INDI::Logger::DBG_DEBUG, "Cannot read data from power sensor");
					PowerReadingsNP.s = IPS_ALERT;
				}	
			}
			else
			{
				DEBUG(INDI::Logger::DBG_DEBUG, "Cannot write data to power sensor");
				PowerReadingsNP.s = IPS_ALERT;
			}			
		}
		powerIndex++;
		if(powerIndex > 5) powerIndex = 0;

		lgI2cClose(i2cHandle);
        IDSetNumber(&PowerReadingsNP, nullptr);		
		return true;
	}
	else
	{
		DEBUG(INDI::Logger::DBG_DEBUG, "No power sensor found.");
		return false;
	}
}

int AstroLink4Pi::checkRevision()
{
	int handle = lgGpiochipOpen(RP5_GPIO);

	if(handle < 0)
	{
		handle = lgGpiochipOpen(RP4_GPIO);
		if(handle < 0)
		{
			DEBUG(INDI::Logger::DBG_SESSION, "Neither RPi4 nor RPi5 GPIO was detected.\n");
		}
		else
		{
			gpioType = RP4_GPIO;
		}
	}
	else
	{
		gpioType = RP5_GPIO;
	}

	lgChipInfo_t cInfo;
	int status = lgGpioGetChipInfo(handle, &cInfo);

	if (status == LG_OKAY)
	{
		DEBUGF(INDI::Logger::DBG_SESSION, "GPIO chip lines=%d name=%s label=%s\n", cInfo.lines, cInfo.name, cInfo.label);
	}

	int rev = 1;
	lgGpioClaimOutput(handle, 0, MOTOR_PWM, 0);
	lgGpioClaimInput(handle, 0, CHK_IN_PIN);
	int result = lgGpioRead(handle, CHK_IN_PIN);
	if (result == 0)
	{
		result = lgGpioWrite(handle, MOTOR_PWM, 1);
		result = lgGpioRead(handle, CHK_IN_PIN);
		if(result == 1)
		{
			rev = 4;
		}
	}
	lgTxPwm(handle, MOTOR_PWM, 5000, 0, 0, 0);
	lgGpioFree(handle, MOTOR_PWM);
	lgGpioFree(handle, CHK_IN_PIN);

	lgGpiochipClose(handle);

	DEBUGF(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi revision %d detected", rev);
	return rev;
}