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

#include "astrolink4pi.h"

auto astroLink4Pi = std::make_unique<AstroLink4Pi>();

static constexpr int POLL_PERIOD = 200;
static constexpr int SENSOR_READ_PERIOD = 1000;

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
	ISInit();
	astroLink4Pi->ISNewNumber(dev, name, values, names, num);
}

AstroLink4Pi::AstroLink4Pi() : FI(this), WI(this)
, m_BoardIO(getDeviceName())
, m_PwmController(m_BoardIO, (getDeviceName()))
, m_SystemInfo(getDeviceName())
, m_PowerMonitor(getDeviceName())
, m_SHTReader(getDeviceName())
, m_MLXReader(getDeviceName())
, m_TSLReader(getDeviceName())
, m_Focuser(Focuser::Config{}, m_BoardIO, m_PwmController, getDeviceName())
{
	setVersion(VERSION_MAJOR, VERSION_MINOR);
}

AstroLink4Pi::~AstroLink4Pi()
{
	// if (_motionThread.joinable())
	// {
	// 	//_abort = true;
	// 	_abort.store(true, std::memory_order_relaxed);
	// 	_motionThread.join();
	// }
}

const char *AstroLink4Pi::getDefaultName()
{
	return (char *)"AstroLink 4 Pi";
}

bool AstroLink4Pi::Connect()
{
	// Required modules
	if (!m_BoardIO.connect())
	{
		DEBUG(INDI::Logger::DBG_ERROR, "Could not access GPIO.");
		return false;
	}
	if (!m_PwmController.connect())
	{
		DEBUG(INDI::Logger::DBG_ERROR, "Could not initialize PWM.");
		return false;
	}
	if (!m_Focuser.open(m_BoardIO.revision()))
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Could not initialize Focuser module.");
		return false;
	}

	DEBUGF(INDI::Logger::DBG_SESSION,
		   "Connected on %s (%s), kernel %s",
		   m_SystemInfo.getHostname().c_str(),
		   m_SystemInfo.getModel().c_str(),
		   m_SystemInfo.getKernelVersion().c_str());


	// Optional modules
	if (!m_PowerMonitor.open())
	{
		DEBUG(INDI::Logger::DBG_DEBUG, "Power monitor not detected.");
	}
	if (!m_SHTReader.open())
	{
		DEBUG(INDI::Logger::DBG_DEBUG, "SHT sensor not detected.");
	}
	if (!m_MLXReader.open())
	{
		DEBUG(INDI::Logger::DBG_DEBUG, "MLX sensor not detected.");
	}
	if (!m_TSLReader.open())
	{
		DEBUG(INDI::Logger::DBG_DEBUG, "TSL sensor not detected.");
	}

	// Lock Relay Labels setting
	RelayLabelsTP.s = IPS_BUSY;
	IDSetText(&RelayLabelsTP, nullptr);

	// update Hardware
	// https://www.raspberrypi.org/documentation/hardware/raspberrypi/revision-codes/README.md

	IUSaveText(&SysInfoT[SYSI_HOST], m_SystemInfo.getHostname().c_str());
	IUSaveText(&SysInfoT[SYSI_HARDWARE], m_SystemInfo.getModel().c_str());
	IUSaveText(&SysInfoT[SYSI_LOCALIP], m_SystemInfo.getLocalIP().c_str());

	// Update client
	IDSetText(&SysInfoTP, NULL);

	// read last position from file & convert from MAX_RESOLUTION to current resolution
	FocusAbsPosNP[0].setValue(m_Focuser.getState().currentPosition);

	getFocuserInfo();

	SetTimer(POLL_PERIOD);

	DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi connected successfully.");

	return true;
}

bool AstroLink4Pi::Disconnect()
{
	m_Focuser.close();
	m_PwmController.shutdown();
	m_PowerMonitor.close();
	m_BoardIO.disconnect();

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
	WI::initProperties(SYSTEM_TAB, ENVIRONMENT_TAB);

	// addDebugControl();
	// addSimulationControl();
	addConfigurationControl();

	// Focuser Resolution
	IUFillSwitch(&FocusResolutionS[RES_1], "RES_1", "Full Step", ISS_ON);
	IUFillSwitch(&FocusResolutionS[RES_2], "RES_2", "Half Step", ISS_OFF);
	IUFillSwitch(&FocusResolutionS[RES_4], "RES_4", "1/4 STEP", ISS_OFF);
	IUFillSwitch(&FocusResolutionS[RES_8], "RES_8", "1/8 STEP", ISS_OFF);
	IUFillSwitch(&FocusResolutionS[RES_16], "RES_16", "1/16 STEP", ISS_OFF);
	IUFillSwitch(&FocusResolutionS[RES_32], "RES_32", "1/32 STEP", ISS_OFF);
	IUFillSwitchVector(&FocusResolutionSP, FocusResolutionS, 6, getDeviceName(), "FOCUS_RESOLUTION", "Resolution", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Focuser motor hold
	IUFillSwitch(&FocusHoldS[HOLD_0], "HOLD_0", "0%", ISS_ON);
	IUFillSwitch(&FocusHoldS[HOLD_20], "HOLD_20", "20%", ISS_OFF);
	IUFillSwitch(&FocusHoldS[HOLD_40], "HOLD_40", "40%", ISS_OFF);
	IUFillSwitch(&FocusHoldS[HOLD_60], "HOLD_60", "60%", ISS_OFF);
	IUFillSwitch(&FocusHoldS[HOLD_80], "HOLD_80", "80%", ISS_OFF);
	IUFillSwitch(&FocusHoldS[HOLD_100], "HOLD_100", "100%", ISS_OFF);
	IUFillSwitchVector(&FocusHoldSP, FocusHoldS, 6, getDeviceName(), "FOCUS_HOLD", "Hold power", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Step delay setting
	IUFillNumber(&FocusStepDelayN[0], "FOCUS_STEPDELAY_VALUE", "microseconds", "%0.0f", 200, 20000, 1, 2000);
	IUFillNumberVector(&FocusStepDelayNP, FocusStepDelayN, 1, getDeviceName(), "FOCUS_STEPDELAY", "Step Delay", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Focuser temperature
	IUFillNumber(&FocusTemperatureN[0], "FOCUS_TEMPERATURE_VALUE", "°C", "%0.2f", -50, 50, 1, 0);
	IUFillNumberVector(&FocusTemperatureNP, FocusTemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

	// Temperature Coefficient
	IUFillNumber(&TemperatureCoefN[0], "steps/C", "", "%.1f", -1000, 1000, 1, 0);
	IUFillNumberVector(&TemperatureCoefNP, TemperatureCoefN, 1, getDeviceName(), "Temperature Coefficient", "", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Compensate for temperature
	IUFillSwitch(&TemperatureCompensateS[0], "Enable", "", ISS_OFF);
	IUFillSwitch(&TemperatureCompensateS[1], "Disable", "", ISS_ON);
	IUFillSwitchVector(&TemperatureCompensateSP, TemperatureCompensateS, 2, getDeviceName(), "Temperature Compensate", "", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Focuser Info
	IUFillNumber(&FocuserInfoN[FOC_STEP_SIZE], "FOC_STEP_SIZE", "Step Size (um)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[FOC_CFZ], "FOC_CFZ", "Critical Focus Zone (um)", "%0.2f", 0, 1000, 1, 0);
	IUFillNumber(&FocuserInfoN[FOC_STEPS_CFZ], "FOC_STEPS_CFZ", "Steps / Critical Focus Zone", "%0.0f", 0, 1000, 1, 0);
	IUFillNumberVector(&FocuserInfoNP, FocuserInfoN, 3, getDeviceName(), "FOCUSER_PARAMETERS", "Focuser Info", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

	// Maximum focuser travel
	IUFillNumber(&FocuserTravelN[0], "FOCUSER_TRAVEL_VALUE", "mm", "%0.0f", 10, 200, 10, 10);
	IUFillNumberVector(&FocuserTravelNP, FocuserTravelN, 1, getDeviceName(), "FOCUSER_TRAVEL", "Max Travel", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	// Scope params
	IUFillNumber(&ScopeParametersN[SCOPE_DIAM], "SCOPE_DIAM", "Aperture (mm)", "%0.0f", 10, 5000, 0, 0.0);
	IUFillNumber(&ScopeParametersN[SCOPE_FL], "SCOPE_FL", "Focal Length (mm)", "%0.0f", 10, 10000, 0, 0.0);
	IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, getDeviceName(), "TELESCOPE_INFO", "Scope Properties", OPTIONS_TAB, IP_RW, 60, IPS_OK);

	IUFillText(&SysTimeT[SYST_TIME], "SYST_TIME", "Local Time", NULL);
	IUFillText(&SysTimeT[SYST_OFFSET], "SYST_OFFSET", "UTC Offset", NULL);
	IUFillTextVector(&SysTimeTP, SysTimeT, 2, getDeviceName(), "SYSTEM_TIME", "System Time", SYSTEM_TAB, IP_RO, 60, IPS_IDLE);

	IUFillText(&SysInfoT[SYSI_HARDWARE], "SYSI_HARDWARE", "Hardware", NULL);
	IUFillText(&SysInfoT[SYSI_CPUTEMP], "SYSI_CPUTEMP", "CPU Temp (°C)", NULL);
	IUFillText(&SysInfoT[SYSI_UPTIME], "SYSI_UPTIME", "Uptime (hh:mm)", NULL);
	IUFillText(&SysInfoT[SYSI_LOAD], "SYSI_LOAD", "Load (1 / 5 / 15 min.)", NULL);
	IUFillText(&SysInfoT[SYSI_HOST], "SYSI_HOST", "Hostname", NULL);
	IUFillText(&SysInfoT[SYSI_LOCALIP], "SYSI_LOCALIP", "Local IP", NULL);
	IUFillTextVector(&SysInfoTP, SysInfoT, 6, getDeviceName(), "SYSTEM_INFO", "System Info", SYSTEM_TAB, IP_RO, 60, IPS_IDLE);

	IUFillNumber(&FanPowerN[0], "FAN_PWR", "Speed [%]", "%0.0f", 0, 100, 1, 33);
	IUFillNumberVector(&FanPowerNP, FanPowerN, 1, getDeviceName(), "FAN_POWER", "Internal fan", SYSTEM_TAB, IP_RO, 60, IPS_IDLE);

	IUFillText(&RelayLabelsT[LAB_OUT1], "LAB_OUT1", "OUT 1", "OUT 1");
	IUFillText(&RelayLabelsT[LAB_OUT2], "LAB_OUT2", "OUT 2", "OUT 2");
	IUFillText(&RelayLabelsT[LAB_PWM1], "LAB_PWM1", "PWM 1", "PWM 1");
	IUFillText(&RelayLabelsT[LAB_PWM2], "LAB_PWM2", "PWM 2", "PWM 2");
	IUFillTextVector(&RelayLabelsTP, RelayLabelsT, 4, getDeviceName(), "RELAYLABELS", "Relay Labels", OPTIONS_TAB, IP_RW, 60, IPS_IDLE);

	IUFillNumber(&SQMOffsetN[0], "SQMOffset", "mag/arcsec2", "%0.2f", -1, 1, 0.01, 0);
	IUFillNumberVector(&SQMOffsetNP, SQMOffsetN, 1, getDeviceName(), "SQMOFFSET", "SQM calibration", OPTIONS_TAB, IP_RW, 60, IPS_IDLE);

	// Load options before connecting
	// load config before defining switches
	defineProperty(&RelayLabelsTP);
	loadConfig();

	IUFillNumber(&StepperCurrentN[0], "STEPPER_CURRENT", "mA", "%0.0f", 200, 2000, 100, 400);
	IUFillNumberVector(&StepperCurrentNP, StepperCurrentN, 1, getDeviceName(), "STEPPER_CURRENT", "Stepper current", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	IUFillSwitch(&Switch1S[S1_ON], "S1_ON", "ON", ISS_OFF);
	IUFillSwitch(&Switch1S[S1_OFF], "S1_OFF", "OFF", ISS_ON);
	IUFillSwitchVector(&Switch1SP, Switch1S, 2, getDeviceName(), "SWITCH_1", RelayLabelsT[0].text, OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillSwitch(&Switch2S[S2_ON], "S2_ON", "ON", ISS_OFF);
	IUFillSwitch(&Switch2S[S2_OFF], "S2_OFF", "OFF", ISS_ON);
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
	FocusMaxPosNP[0].setMin(1000);
	FocusMaxPosNP[0].setMax(100000);
	FocusMaxPosNP[0].setStep(1000);
	FocusMaxPosNP[0].setValue(10000);

	FocusRelPosNP[0].setMin(0);
	FocusRelPosNP[0].setMax(10000);
	FocusRelPosNP[0].setStep(100);
	FocusRelPosNP[0].setValue(100);

	FocusAbsPosNP[0].setMin(0);
	FocusAbsPosNP[0].setMax(FocusMaxPosNP[0].getValue());
	FocusAbsPosNP[0].setStep((int)FocusAbsPosNP[0].getMax() / 100);

	FocusMotionSP[FOCUS_OUTWARD].setState(ISS_ON);
	FocusMotionSP[FOCUS_INWARD].setState(ISS_OFF);
	FocusMotionSP.apply();

	return true;
}

bool AstroLink4Pi::updateProperties()
{
	INDI::DefaultDevice::updateProperties();

	if (isConnected())
	{
		FI::updateProperties();
		WI::updateProperties();

		defineProperty(&ScopeParametersNP);
		defineProperty(&FocuserTravelNP);
		defineProperty(&FocusResolutionSP);
		defineProperty(&FocusHoldSP);
		defineProperty(&FocuserInfoNP);
		defineProperty(&FocusStepDelayNP);
		defineProperty(&SysTimeTP);
		defineProperty(&SysInfoTP);
		defineProperty(&Switch1SP);
		defineProperty(&Switch2SP);
		defineProperty(&PWM1NP);
		defineProperty(&PWM2NP);
		defineProperty(&StepperCurrentNP);
		defineProperty(&FocusTemperatureNP);
		defineProperty(&TemperatureCoefNP);
		defineProperty(&TemperatureCompensateSP);
		defineProperty(&PowerReadingsNP);
		defineProperty(&FanPowerNP);
		defineProperty(&SQMOffsetNP);
	}
	else
	{
		deleteProperty(SQMOffsetNP.name);
		deleteProperty(ScopeParametersNP.name);
		deleteProperty(FocuserTravelNP.name);
		deleteProperty(FocusResolutionSP.name);
		deleteProperty(FocusHoldSP.name);
		deleteProperty(FocuserInfoNP.name);
		deleteProperty(FocusStepDelayNP.name);
		deleteProperty(FocusTemperatureNP.name);
		deleteProperty(TemperatureCoefNP.name);
		deleteProperty(TemperatureCompensateSP.name);
		deleteProperty(SysTimeTP.name);
		deleteProperty(SysInfoTP.name);
		deleteProperty(Switch1SP.name);
		deleteProperty(Switch2SP.name);
		deleteProperty(PWM1NP.name);
		deleteProperty(PWM2NP.name);
		deleteProperty(StepperCurrentNP.name);
		deleteProperty(PowerReadingsNP.name);
		deleteProperty(FanPowerNP.name);
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
		// handle scope params
		if (!strcmp(name, ScopeParametersNP.name))
		{
			ScopeParametersNP.s = IPS_BUSY;
			IUUpdateNumber(&ScopeParametersNP, values, names, n);
			IDSetNumber(&FocusStepDelayNP, nullptr);
			ScopeParametersNP.s = IPS_OK;
			IDSetNumber(&ScopeParametersNP, nullptr);
			getFocuserInfo();
			DEBUGF(INDI::Logger::DBG_SESSION, "Scope parameters set to %0.0f / %0.0f.", ScopeParametersN[SCOPE_DIAM].value, ScopeParametersN[SCOPE_FL].value);
			return true;
		}

		// handle focus step delay
		if (!strcmp(name, FocusStepDelayNP.name))
		{
			FocusStepDelayNP.s = IPS_BUSY;
			IUUpdateNumber(&FocusStepDelayNP, values, names, n);
			IDSetNumber(&FocusStepDelayNP, nullptr);
			FocusStepDelayNP.s = IPS_OK;
			IDSetNumber(&FocusStepDelayNP, nullptr);
			m_Focuser.setStepDelayUs(FocusStepDelayN[0].value);
			DEBUGF(INDI::Logger::DBG_SESSION, "Step delay set to %0.0f us.", FocusStepDelayN[0].value);
			return true;
		}

		// handle focus maximum position
		if (!strcmp(name, FocusMaxPosNP.getName()))
		{
			FocusMaxPosNP.update(values, names, n);

			FocusAbsPosNP[0].setMax(FocusMaxPosNP[0].getValue());
			FocusAbsPosNP.updateMinMax(); // This call is not INDI protocol compliant

			FocusMaxPosNP.setState(IPS_OK);
			FocusMaxPosNP.apply();
			getFocuserInfo();
			return true;
		}

		// handle temperature coefficient
		if (!strcmp(name, TemperatureCoefNP.name))
		{
			IUUpdateNumber(&TemperatureCoefNP, values, names, n);
			TemperatureCoefNP.s = IPS_OK;
			IDSetNumber(&TemperatureCoefNP, nullptr);
			m_Focuser.setTemperatureCoefficient(TemperatureCoefN[0].value);
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
			m_PwmController.setDutyPercent(PwmController::Channel::P1, PWM1N[0].value);
			(PWM1N[0].value > 0) ? m_PwmController.enable(PwmController::Channel::P1) : m_PwmController.disable(PwmController::Channel::P1);
			// pwmState[0] = PWM1N[0].value;
			DEBUGF(INDI::Logger::DBG_SESSION, "PWM 1 set to %0.0f", PWM1N[0].value);
			return true;
		}

		if (!strcmp(name, PWM2NP.name))
		{
			IUUpdateNumber(&PWM2NP, values, names, n);
			PWM2NP.s = IPS_OK;
			IDSetNumber(&PWM2NP, nullptr);
			m_PwmController.setDutyPercent(PwmController::Channel::P2, PWM2N[0].value);
			(PWM2N[0].value > 0) ? m_PwmController.enable(PwmController::Channel::P2) : m_PwmController.disable(PwmController::Channel::P2);
			// pwmState[1] = PWM2N[0].value;
			DEBUGF(INDI::Logger::DBG_SESSION, "PWM 2 set to %0.0f", PWM2N[0].value);
			return true;
		}

		// SQM calibration
		if (!strcmp(name, SQMOffsetNP.name))
		{
			SQMOffsetNP.s = IPS_BUSY;
			IUUpdateNumber(&SQMOffsetNP, values, names, n);
			SQMOffsetNP.s = IPS_OK;
			IDSetNumber(&SQMOffsetNP, nullptr);
			return true;
		}

		// handle stepper current
		if (!strcmp(name, StepperCurrentNP.name))
		{
			IUUpdateNumber(&StepperCurrentNP, values, names, n);
			StepperCurrentNP.s = IPS_OK;
			IDSetNumber(&StepperCurrentNP, nullptr);
			m_Focuser.setCurrent(static_cast<int>(StepperCurrentN[0].value));
			DEBUGF(INDI::Logger::DBG_SESSION, "Stepper current set to %0.0f mA", StepperCurrentN[0].value);
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
				m_Focuser.setTemperatureCompensation(true);
				DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation ENABLED.");
			}

			if (TemperatureCompensateS[1].s == ISS_ON)
			{
				TemperatureCompensateSP.s = IPS_IDLE;
				m_Focuser.setTemperatureCompensation(false);
				DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation DISABLED.");
			}

			IDSetSwitch(&TemperatureCompensateSP, nullptr);
			return true;
		}

		// handle relay 1
		if (!strcmp(name, Switch1SP.name))
		{
			IUUpdateSwitch(&Switch1SP, states, names, n);

			if (Switch1S[S1_ON].s == ISS_ON)
			{
				if (!m_BoardIO.setOut1(1))
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting AstroLink Relay #1");
					Switch1SP.s = IPS_ALERT;
					Switch1S[S1_ON].s = ISS_OFF;
					IDSetSwitch(&Switch1SP, NULL);
					return false;
				}
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink Relays #1 set to ON");
				Switch1SP.s = IPS_OK;
				Switch1S[S1_OFF].s = ISS_OFF;
				IDSetSwitch(&Switch1SP, NULL);
				return true;
			}
			if (Switch1S[S1_OFF].s == ISS_ON)
			{
				if (!m_BoardIO.setOut1(0))
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting AstroLink Relay #1");
					Switch1SP.s = IPS_ALERT;
					Switch1S[S1_ON].s = ISS_OFF;
					IDSetSwitch(&Switch1SP, NULL);
					return false;
				}
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink Relays #1 set to OFF");
				Switch1SP.s = IPS_IDLE;
				Switch1S[S1_ON].s = ISS_OFF;
				IDSetSwitch(&Switch1SP, NULL);
				return true;
			}
		}

		// handle relay 2
		if (!strcmp(name, Switch2SP.name))
		{
			IUUpdateSwitch(&Switch2SP, states, names, n);

			if (Switch2S[S2_ON].s == ISS_ON)
			{
				if (!m_BoardIO.setOut2(1))
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting AstroLink Relay #2");
					Switch2SP.s = IPS_ALERT;
					Switch2S[S1_ON].s = ISS_OFF;
					IDSetSwitch(&Switch2SP, NULL);
					return false;
				}
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink Relays #2 set to ON");
				Switch2SP.s = IPS_OK;
				Switch2S[S2_OFF].s = ISS_OFF;
				IDSetSwitch(&Switch2SP, NULL);
				return true;
			}
			if (Switch2S[S2_OFF].s == ISS_ON)
			{
				if (!m_BoardIO.setOut2(0))
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting AstroLink Relay #2");
					Switch2SP.s = IPS_ALERT;
					Switch2S[S1_ON].s = ISS_OFF;
					IDSetSwitch(&Switch2SP, NULL);
					return false;
				}
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink Relays #2 set to OFF");
				Switch2SP.s = IPS_IDLE;
				Switch2S[S2_ON].s = ISS_OFF;
				IDSetSwitch(&Switch2SP, NULL);
				return true;
			}
		}

		// handle focus motor hold
		if (!strcmp(name, FocusHoldSP.name))
		{
			IUUpdateSwitch(&FocusHoldSP, states, names, n);
			FocusHoldSP.s = IPS_OK;
			m_Focuser.setHoldPowerPercent(20 * getHoldPower());
			IDSetSwitch(&FocusHoldSP, nullptr);
			return true;
		}

		// handle focus resolution
		if (!strcmp(name, FocusResolutionSP.name))
		{
			int resolution = 1;
			IUUpdateSwitch(&FocusResolutionSP, states, names, n);

			if (FocusResolutionS[RES_1].s == ISS_ON)
				resolution = 1;

			if (FocusResolutionS[RES_2].s == ISS_ON)
				resolution = 2;

			if (FocusResolutionS[RES_4].s == ISS_ON)
				resolution = 4;

			if (FocusResolutionS[RES_8].s == ISS_ON)
				resolution = 8;

			if (FocusResolutionS[RES_16].s == ISS_ON)
				resolution = 16;

			if (FocusResolutionS[RES_32].s == ISS_ON)
				resolution = 32;
			m_Focuser.setResolution(resolution);
			getFocuserInfo();

			FocusResolutionSP.s = IPS_OK;
			IDSetSwitch(&FocusResolutionSP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Resolution set to 1 / %d.", resolution);
			return true;
		}

		if (strstr(name, "FOCUS"))
			return FI::processSwitch(dev, name, states, names, n);
		if (strstr(name, "WEATHER_"))
			return WI::processSwitch(dev, name, states, names, n);
	}

	return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool AstroLink4Pi::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
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

bool AstroLink4Pi::saveConfigItems(FILE *fp)
{
	FI::saveConfigItems(fp);
	WI::saveConfigItems(fp);
	IUSaveConfigSwitch(fp, &FocusResolutionSP);
	IUSaveConfigSwitch(fp, &FocusHoldSP);
	IUSaveConfigSwitch(fp, &TemperatureCompensateSP);
	IUSaveConfigNumber(fp, &FocusStepDelayNP);
	IUSaveConfigNumber(fp, &FocuserTravelNP);
	IUSaveConfigNumber(fp, &ScopeParametersNP);
	IUSaveConfigNumber(fp, &TemperatureCoefNP);
	IUSaveConfigText(fp, &RelayLabelsTP);
	IUSaveConfigSwitch(fp, &Switch1SP);
	IUSaveConfigSwitch(fp, &Switch2SP);
	IUSaveConfigNumber(fp, &StepperCurrentNP);
	IUSaveConfigNumber(fp, &PWM1NP);
	IUSaveConfigNumber(fp, &PWM2NP);
	IUSaveConfigNumber(fp, &SQMOffsetNP);

	return true;
}

void AstroLink4Pi::TimerHit()
{
	if (!isConnected())
		return;

	uint64_t timeMillis = m_SystemInfo.millis();
	readTSL();
	// readPower();
	// focuserUpdate();

	if (nextSystemRead < timeMillis)
	{
		m_Cycle = next(m_Cycle);

		switch (m_Cycle)
		{
		case SensorCycle::SHT_T:
			readSHT(0);
			break;
		case SensorCycle::SHT_R:
			readSHT(1);
			break;
		case SensorCycle::MLX:
			readMLX();
			break;
		case SensorCycle::SYS:
			systemUpdate();
			break;
		case SensorCycle::FAN:
			// fanUpdate();
			break;
		case SensorCycle::COMP:
			// if (FocusTemperatureNP.s == IPS_OK)
			// {
			// 	m_Focuser.setTemperature(FocusTemperatureN[0].value);
			// 	m_Focuser.temperatureCompensation();
			// }
			break;
		default:
			break;
		}
		nextSystemRead = timeMillis + SENSOR_READ_PERIOD;
	}

	SetTimer(POLL_PERIOD);
}

bool AstroLink4Pi::AbortFocuser()
{
	return m_Focuser.abortFocuser();
}

IPState AstroLink4Pi::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t current = (int32_t)FocusAbsPosNP[0].getValue();
    int32_t delta = (dir == FOCUS_INWARD ? -1 : 1) * (int32_t)ticks;

    int32_t target = current + delta;

    if (target < 0)
        target = 0;
    if (target > (int32_t)FocusAbsPosNP[0].getMax())
        target = (int32_t)FocusAbsPosNP[0].getMax();

    return MoveAbsFocuser((uint32_t)target);
}

IPState AstroLink4Pi::MoveAbsFocuser(uint32_t targetTicks)
{
	if (targetTicks == FocusAbsPosNP[0].getValue())
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Already at the requested position.");
		return IPS_OK;
	}

	m_Focuser.setCurrent(false);
	return (m_Focuser.moveAbsFocuser(targetTicks) ? IPS_BUSY : IPS_ALERT);
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
	m_Focuser.reverseFocuser(enabled);
	return true;
}

bool AstroLink4Pi::SyncFocuser(uint32_t ticks)
{
	if (m_Focuser.syncFocuser(ticks))
	{
		FocusAbsPosNP[0].setValue(ticks);
		FocusAbsPosNP.apply();
		DEBUGF(INDI::Logger::DBG_SESSION, "Absolute Position synced to %0.0f", FocusAbsPosNP[0].getValue());
		return true;
	}
	else
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Could not sync focuser position");
		return false;
	}
}

bool AstroLink4Pi::SetFocuserBacklash(int32_t steps)
{
	m_Focuser.setFocuserBacklash(steps);
	DEBUGF(INDI::Logger::DBG_SESSION, "Backlash set to %i steps", steps);
	return true;
}

bool AstroLink4Pi::SetFocuserMaxPosition(uint32_t ticks)
{
	m_Focuser.setFocuserMaxPosition(ticks);
	DEBUGF(INDI::Logger::DBG_SESSION, "Max position set to %i steps", ticks);
	return true;
}

bool AstroLink4Pi::readDS18B20()
{
	if (!isConnected())
		return false;

	// if (m_DSReader.open())
	// {
	// 	DSFileReader::Readings r;
	// 	if (m_DSReader.read(r))
	// 	{
	// 		double tempC = r.temperature;
	// 		setParameterValue("WEATHER_TEMPERATURE", tempC);
	// 		DEBUGF(INDI::Logger::DBG_DEBUG, "Temperature: %.2f°C", tempC);
	// 		focuserTemperature = tempC;
	// 		return true;
	// 	}
	// }
	return false;
}

int AstroLink4Pi::getHoldPower()
{
	if (FocusHoldS[HOLD_20].s == ISS_ON)
		return 1;
	if (FocusHoldS[HOLD_40].s == ISS_ON)
		return 2;
	if (FocusHoldS[HOLD_60].s == ISS_ON)
		return 3;
	if (FocusHoldS[HOLD_80].s == ISS_ON)
		return 4;
	if (FocusHoldS[HOLD_100].s == ISS_ON)
		return 5;
	return 0;
}

void AstroLink4Pi::focuserUpdate()
{
	auto state = m_Focuser.getState();
	FocusAbsPosNP[0].setValue(state.currentPosition);
	FocusAbsPosNP.setState(state.moving ? IPS_BUSY : IPS_OK);
	FocusAbsPosNP.apply();
	FocusRelPosNP.setState(state.moving ? IPS_BUSY : IPS_OK);
	FocusRelPosNP.apply();
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
	IUSaveText(&SysTimeT[SYST_TIME], ts);
	snprintf(ts, sizeof(ts), "%4.2f", (local_timeinfo->tm_gmtoff / 3600.0));
	IUSaveText(&SysTimeT[SYST_OFFSET], ts);
	SysTimeTP.s = IPS_OK;
	IDSetText(&SysTimeTP, NULL);

	SysInfoTP.s = IPS_BUSY;
	IDSetText(&SysInfoTP, NULL);

	// update CPU temp
	IUSaveText(&SysInfoT[SYSI_CPUTEMP], m_SystemInfo.getCpuTemp().c_str());

	// update uptime
	IUSaveText(&SysInfoT[SYSI_UPTIME], m_SystemInfo.getUptimeString().c_str());

	// update load
	IUSaveText(&SysInfoT[SYSI_LOAD], m_SystemInfo.getLoad().c_str());

	SysInfoTP.s = IPS_OK;
	IDSetText(&SysInfoTP, NULL);
}

void AstroLink4Pi::getFocuserInfo()
{
	// https://www.innovationsforesight.com/education/how-much-focus-error-is-too-much/
	float travel_mm = (float)FocuserTravelN[0].value;
	float aperture = (float)ScopeParametersN[SCOPE_DIAM].value;
	float focal = (float)ScopeParametersN[SCOPE_FL].value;
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

	float cfz = 4.88 * 0.520 * pow(f_ratio, 2); // CFZ = 4.88 Â· Î» Â· f^2
	float step_size = 1000.0 * travel_mm / FocusMaxPosNP[0].getValue();
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

	FocuserInfoN[FOC_STEP_SIZE].value = step_size;
	FocuserInfoN[FOC_CFZ].value = cfz;
	FocuserInfoN[FOC_STEPS_CFZ].value = steps_per_cfz;
	IDSetNumber(&FocuserInfoNP, nullptr);

	DEBUGF(INDI::Logger::DBG_DEBUG, "Focuser Info: %0.2f %0.2f %0.2f.", FocuserInfoN[0].value, FocuserInfoN[1].value, FocuserInfoN[2].value);
}

void AstroLink4Pi::fanUpdate()
{
	const char *txt = SysInfoT[SYSI_CPUTEMP].text;

	FanPowerNP.s = IPS_BUSY;
	if (txt && *txt) // != nullptr i nie pusty string
	{
		int temp = 0;
		int cycle = 0;
		try
		{
			temp = std::stoi(txt);
			double fanPwr = 33.0;
			if (temp > 60)
			{
				cycle = 50;
				fanPwr = 66.0;
			}
			if (temp > 70)
			{
				cycle = 100;
				fanPwr = 100.0;
			}
			m_PwmController.setDutyPercent(PwmController::Channel::FAN, cycle);
			FanPowerN[0].value = fanPwr;
			FanPowerNP.s = IPS_OK;
		}
		catch (const std::exception &)
		{
			FanPowerNP.s = IPS_ALERT;
		}
	}
	IDSetNumber(&FanPowerNP, nullptr);
}

bool AstroLink4Pi::readTSL()
{
	if(!m_TSLReader.isOpen()) return false;

	TSLReader::Readings readings;
	m_TSLReader.setSQMOffset(SQMOffsetN[0].value);
	bool correct = m_TSLReader.read(readings) && readings.valid;
	if (!correct)
	{
		readings = TSLReader::Readings{};
	}
	setParameterValue("SQM_READING", readings.mpsas);
	return correct;
}

bool AstroLink4Pi::readMLX()
{
	if(!m_MLXReader.isOpen()) return false;

	MLXReader::Readings readings;
	bool correct = m_MLXReader.read(readings);
	if (!correct)
	{
		readings = MLXReader::Readings{};
	}
	setParameterValue("WEATHER_SKY_TEMP", readings.objectTemperature);
	setParameterValue("WEATHER_SKY_DIFF", readings.tempDifference);

	return correct;
}

bool AstroLink4Pi::readSHT(int mode)
{
	if(!m_SHTReader.isOpen()) return false;

	SHTReader::Readings readings;
	bool correct = m_SHTReader.read(readings, mode);
	if (correct)
	{
		FocusTemperatureN[0].value = readings.temperature;
		FocusTemperatureNP.s = IPS_OK;
	}
	else
	{
		readings = SHTReader::Readings{};
		FocusTemperatureN[0].value = 0.0;
		FocusTemperatureNP.s = IPS_ALERT;
	}
	IDSetNumber(&FocusTemperatureNP, nullptr);

	setParameterValue("WEATHER_TEMPERATURE", readings.temperature);
	setParameterValue("WEATHER_HUMIDITY", readings.humidity);
	setParameterValue("WEATHER_DEWPOINT", readings.dewPoint);


	return correct;
}

bool AstroLink4Pi::readPower()
{
	PowerMonitor::Readings readings;
	if(!m_PowerMonitor.isOpen()) return false;

	bool correct = m_PowerMonitor.read(readings);

	if (correct)
	{
		PowerReadingsNP.s = IPS_OK;

	}
	else 
	{
		readings = PowerMonitor::Readings{};
		PowerReadingsNP.s = IPS_ALERT;
	}

	PowerReadingsN[POW_VIN].value = readings.vin;
	PowerReadingsN[POW_VREG].value = readings.vreg;
	PowerReadingsN[POW_ITOT].value = readings.current;
	PowerReadingsN[POW_PTOT].value = readings.power;
	PowerReadingsN[POW_AH].value = readings.ah;
	PowerReadingsN[POW_WH].value = readings.wh;

	IDSetNumber(&PowerReadingsNP, nullptr);
	return correct;
}
