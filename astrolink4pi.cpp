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
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <math.h>
#include <memory>
#include <time.h>
#include "config.h"


#include <gpiod.h>

#include "astrolink4pi.h"


std::unique_ptr<AstroLink4Pi> astroLink4Pi(new AstroLink4Pi());

#define MAX_RESOLUTION                      32   // the highest resolution supported is 1/32 step
#define TEMPERATURE_UPDATE_TIMEOUT          (5 * 1000) // 3 sec
#define TEMPERATURE_COMPENSATION_TIMEOUT    (30 * 1000) // 60 sec
#define SYSTEM_UPDATE_PERIOD                1000
#define PWM_CYCLE_PERIOD                    100

#define EN_PIN	    15
#define M0_PIN	    17
#define M1_PIN	    18
#define M2_PIN	    27
#define RST_PIN	    22
#define STP_PIN	    24
#define DIR_PIN	    23
#define OUT1_PIN	5
#define OUT2_PIN	6
#define PWM1_PIN	26
#define PWM2_PIN	19
#define HOLD_PIN	10


void ISPoll(void *p);

void ISInit()
{
	static int isInit = 0;

	if (isInit == 1)
	return;
	if(astroLink4Pi.get() == 0)
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

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
        ISInit();
        astroLink4Pi->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
        astroLink4Pi->ISNewNumber(dev, name, values, names, num);
}

void ISSnoopDevice (XMLEle *root)
{
	astroLink4Pi->ISSnoopDevice(root);
}

AstroLink4Pi::AstroLink4Pi() : FI(this)
{
	setVersion(VERSION_MAJOR,VERSION_MINOR);	
}

AstroLink4Pi::~AstroLink4Pi()
{
	//
}

const char * AstroLink4Pi::getDefaultName()
{
        return (char *)"AstroLink 4 Pi";
}

bool AstroLink4Pi::Connect()
{
	chip = gpiod_chip_open("/dev/gpiochip0");
	if (!chip)
	{
		DEBUG(INDI::Logger::DBG_ERROR, "Problem initiating AstroLink 4 Pi.");
		return false;
	}

	// verify BCM Pins are not used by other consumers
	int pins[] = {EN_PIN, M0_PIN, M1_PIN, M2_PIN, RST_PIN, STP_PIN, DIR_PIN, OUT1_PIN, OUT2_PIN, PWM1_PIN, PWM2_PIN, HOLD_PIN};
	for (unsigned int pin = 0; pin < 12; pin++)
	{
		if (gpiod_line_is_used(gpiod_chip_get_line(chip, pins[pin])))
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "BCM Pin %0.0f already used", pins[pin]);
			gpiod_chip_close(chip);
			return false;
		}
	}

	// Select gpios
	gpio_en = gpiod_chip_get_line(chip, EN_PIN);
	gpio_m0 = gpiod_chip_get_line(chip, M0_PIN);
	gpio_m1 = gpiod_chip_get_line(chip, M1_PIN);
	gpio_m2 = gpiod_chip_get_line(chip, M2_PIN);
	gpio_rst = gpiod_chip_get_line(chip, RST_PIN);
	gpio_stp = gpiod_chip_get_line(chip, STP_PIN);
	gpio_dir = gpiod_chip_get_line(chip, DIR_PIN);
	gpio_out1 = gpiod_chip_get_line(chip, OUT1_PIN);
	gpio_out2 = gpiod_chip_get_line(chip, OUT2_PIN);
	gpio_pwm1 = gpiod_chip_get_line(chip, PWM1_PIN);
	gpio_pwm2 = gpiod_chip_get_line(chip, PWM2_PIN);
	gpio_hold = gpiod_chip_get_line(chip, HOLD_PIN);

	// Set initial state for gpios
	gpiod_line_request_output(gpio_en, "en@astrolink4pi_focuser", 1);			// start as disabld
	gpiod_line_request_output(gpio_m0, "m0@astrolink4pi_focuser", 0);
	gpiod_line_request_output(gpio_m1, "m1@astrolink4pi_focuser", 0); 
	gpiod_line_request_output(gpio_m2, "m2@astrolink4pi_focuser", 0);
	gpiod_line_request_output(gpio_rst, "rst@astrolink4pi_focuser", 1);			// start as wake up
	gpiod_line_request_output(gpio_stp, "stp@astrolink4pi_focuser", 0);
	gpiod_line_request_output(gpio_dir, "dir@astrolink4pi_focuser", 0);
	gpiod_line_request_output(gpio_out1, "out1@astrolink4pi_relays", relayState[0]);
	gpiod_line_request_output(gpio_out2, "out2@astrolink4pi_relays", relayState[1]);
	gpiod_line_request_output(gpio_pwm1, "pwm1@astrolink4pi_relays", 0);
	gpiod_line_request_output(gpio_pwm2, "pwm2@astrolink4pi_relays", 0);
	gpiod_line_request_output(gpio_hold, "hold@astrolink4pi_relays", 1);		// start as disabled

    // Lock Relay Labels setting
	RelayLabelsTP.s = IPS_BUSY;
	IDSetText(&RelayLabelsTP, nullptr);

	Switch1DefSP.s = IPS_BUSY;
	Switch2DefSP.s = IPS_BUSY;
	IDSetSwitch(&SwitchDef1SP, nullptr);
	IDSetSwitch(&SwitchDef2SP, nullptr);

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

	//read last position from file & convert from MAX_RESOLUTION to current resolution
	FocusAbsPosN[0].value = savePosition(-1) != -1 ? (int) savePosition(-1) * resolution / MAX_RESOLUTION : 0;

	// preset resolution
	SetResolution(resolution);

    getFocuserInfo();
    long int currentTime = millis();
    nextTemperatureRead = currentTime + TEMPERATURE_UPDATE_TIMEOUT;
    nextTemperatureCompensation = currentTime + TEMPERATURE_COMPENSATION_TIMEOUT;
    nextSystemRead = currentTime + SYSTEM_UPDATE_PERIOD;

    SetTimer(FocusStepDelayN[0].value);

	DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi connected successfully.");

	return true;
}

bool AstroLink4Pi::Disconnect()
{
	// Close device
	gpiod_line_set_value(gpio_hold, 1);
	gpiod_line_set_value(gpio_rst, 0);								// sleep
	int enabledState = gpiod_line_set_value(gpio_en, 1);			// make disabled
	if(enabledState != 0)
	{
		DEBUGF(INDI::Logger::DBG_ERROR, "Cannot set GPIO line %i to disable stepper motor driver. Focusing motor may still be powered.", EN_PIN);
	}
	else
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Focusing motor power disabled.");
	}
	

	gpiod_chip_close(chip);


	// Unlock Relay Labels setting
	RelayLabelsTP.s = IPS_IDLE;
	IDSetText(&RelayLabelsTP, nullptr);    

	SwitchDef1SP.s = IPS_IDLE;
	SwitchDef2SP.s = IPS_IDLE;
	IDSetSwitch(&SwitchDef1SP, nullptr);
	IDSetSwitch(&SwitchDef2SP, nullptr);	
  
	DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi disconnected successfully.");

	return true;
}

bool AstroLink4Pi::initProperties()
{
    INDI::DefaultDevice::initProperties();

    setDriverInterface(AUX_INTERFACE | FOCUSER_INTERFACE);

    FI::SetCapability(FOCUSER_CAN_ABS_MOVE |
                      FOCUSER_CAN_REL_MOVE |
                      FOCUSER_CAN_REVERSE  |
                      FOCUSER_CAN_SYNC     |
                      FOCUSER_CAN_ABORT    |
                      FOCUSER_HAS_BACKLASH);

    FI::initProperties(FOCUS_TAB);

    // addDebugControl();
    // addSimulationControl();
    addConfigurationControl();

	// Focuser Resolution
	IUFillSwitch(&FocusResolutionS[0],"FOCUS_RESOLUTION_1","Full Step",ISS_ON);
	IUFillSwitch(&FocusResolutionS[1],"FOCUS_RESOLUTION_2","Half Step",ISS_OFF);
	IUFillSwitch(&FocusResolutionS[2],"FOCUS_RESOLUTION_4","1/4 STEP",ISS_OFF);
	IUFillSwitch(&FocusResolutionS[3],"FOCUS_RESOLUTION_8","1/8 STEP",ISS_OFF);
	IUFillSwitch(&FocusResolutionS[4],"FOCUS_RESOLUTION_16","1/16 STEP",ISS_OFF);
	IUFillSwitch(&FocusResolutionS[5],"FOCUS_RESOLUTION_32","1/32 STEP",ISS_OFF);
	IUFillSwitchVector(&FocusResolutionSP,FocusResolutionS,6,getDeviceName(),"FOCUS_RESOLUTION","Resolution", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	// Focuser motor hold
	IUFillSwitch(&FocusHoldS[0],"FOCUS_HOLD_0","0%",ISS_ON);
	IUFillSwitch(&FocusHoldS[1],"FOCUS_HOLD_1","50%",ISS_OFF);
	IUFillSwitch(&FocusHoldS[2],"FOCUS_HOLD_2","100%",ISS_OFF);
	IUFillSwitchVector(&FocusHoldSP,FocusHoldS,3,getDeviceName(),"FOCUS_HOLD","Hold power", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Step delay setting
	IUFillNumber(&FocusStepDelayN[0], "FOCUS_STEPDELAY_VALUE", "milliseconds", "%0.0f", 2, 10, 1, 5);
	IUFillNumberVector(&FocusStepDelayNP, FocusStepDelayN, 1, getDeviceName(), "FOCUS_STEPDELAY", "Step Delay", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

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
	IUFillTextVector(&ActiveTelescopeTP, ActiveTelescopeT, 1, getDeviceName(), "ACTIVE_TELESCOPE", "Snoop devices", OPTIONS_TAB,IP_RW, 0, IPS_IDLE);

	// Snooping params
	IUFillNumber(&ScopeParametersN[0], "TELESCOPE_APERTURE", "Aperture (mm)", "%g", 10, 5000, 0, 0.0);
	IUFillNumber(&ScopeParametersN[1], "TELESCOPE_FOCAL_LENGTH", "Focal Length (mm)", "%g", 10, 10000, 0, 0.0);
	IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 2, ActiveTelescopeT[0].text, "TELESCOPE_INFO", "Scope Properties", OPTIONS_TAB, IP_RW, 60, IPS_OK);    
	
	IUFillText(&SysTimeT[0],"LOCAL_TIME","Local Time",NULL);
	IUFillText(&SysTimeT[1],"UTC_OFFSET","UTC Offset",NULL);
	IUFillTextVector(&SysTimeTP,SysTimeT,2,getDeviceName(),"SYSTEM_TIME","System Time",SYSTEM_TAB,IP_RO,60,IPS_IDLE);

	IUFillText(&SysInfoT[0],"HARDWARE","Hardware",NULL);
	IUFillText(&SysInfoT[1],"CPU TEMP","CPU Temp (°C)",NULL);
	IUFillText(&SysInfoT[2],"UPTIME","Uptime (hh:mm)",NULL);
	IUFillText(&SysInfoT[3],"LOAD","Load (1 / 5 / 15 min.)",NULL);
	IUFillText(&SysInfoT[4],"HOSTNAME","Hostname",NULL);
	IUFillText(&SysInfoT[5],"LOCAL_IP","Local IP",NULL);
	IUFillText(&SysInfoT[6],"PUBLIC_IP","Public IP",NULL);
	IUFillTextVector(&SysInfoTP,SysInfoT,7,getDeviceName(),"SYSTEM_INFO","System Info",SYSTEM_TAB,IP_RO,60,IPS_IDLE);

	IUFillSwitch(&SysControlS[0], "SYSCTRL_REBOOT", "Reboot", ISS_OFF);
	IUFillSwitch(&SysControlS[1], "SYSCTRL_SHUTDOWN", "Shutdown", ISS_OFF);
	IUFillSwitchVector(&SysControlSP, SysControlS, 2, getDeviceName(), "SYSCTRL", "System Ctrl", SYSTEM_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillSwitch(&SysOpConfirmS[0], "SYSOPCONFIRM_CONFIRM", "Yes", ISS_OFF);
	IUFillSwitch(&SysOpConfirmS[1], "SYSOPCONFIRM_CANCEL", "No", ISS_OFF);
	IUFillSwitchVector(&SysOpConfirmSP, SysOpConfirmS, 2, getDeviceName(), "SYSOPCONFIRM", "Continue?", SYSTEM_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);    

	IUFillNumber(&PWMcycleN[0], "PWMcycle", "PWM cycle [ms]", "%0.0f", 50, 1000, 50, 100); 
	IUFillNumberVector(&PWMcycleNP, PWMcycleN, 1, getDeviceName(), "PWMCYCLE", "PWM cycle", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

	IUFillText(&RelayLabelsT[0], "RELAYLABEL01", "OUT 1", "OUT 1");
	IUFillText(&RelayLabelsT[1], "RELAYLABEL02", "OUT 2", "OUT 2");
	IUFillText(&RelayLabelsT[2], "RELAYLABEL03", "PWM 1", "PWM 1");
	IUFillText(&RelayLabelsT[3], "RELAYLABEL04", "PWM 2", "PWM 2");
	IUFillTextVector(&RelayLabelsTP, RelayLabelsT, 4, getDeviceName(), "RELAYLABELS", "Relay Labels", OPTIONS_TAB, IP_RW, 60, IPS_IDLE);   
	
	IUFillSwitch(&SwitchDef1S[0], "SW1ON", "ON", ISS_OFF);
	IUFillSwitch(&SwitchDef1S[1], "SW1OFF", "OFF", ISS_ON);
	IUFillSwitchVector(&SwitchDef1SP, SwitchDef1S, 2, getDeviceName(), "SWITCH_1", "Default OUT 1", OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillSwitch(&SwitchDef2S[0], "SW2ON", "ON", ISS_OFF);
	IUFillSwitch(&SwitchDef2S[1], "SW2OFF", "OFF", ISS_ON);
	IUFillSwitchVector(&SwitchDef2SP, SwitchDef2S, 2, getDeviceName(), "SWITCH_2", "Default OUT 1", OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);	

	// Load options before connecting
	// load config before defining switches
    defineProperty(&PWMcycleNP);
	defineProperty(&RelayLabelsTP);
	defineProperty(&SwitchDef1SP);
	defineProperty(&SwitchDef2SP);
	loadConfig();

	IUFillSwitch(&Switch1S[0], "SW1ON", "ON", SwitchDef1S[0].s);
	IUFillSwitch(&Switch1S[1], "SW1OFF", "OFF", SwitchDef1S[1].s);
	IUFillSwitchVector(&Switch1SP, Switch1S, 2, getDeviceName(), "SWITCH_1", RelayLabelsT[0].text, OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillSwitch(&Switch2S[0], "SW2ON", "ON", SwitchDef2S[0].s);
	IUFillSwitch(&Switch2S[1], "SW2OFF", "OFF", SwitchDef2S[1].s);
	IUFillSwitchVector(&Switch2SP, Switch2S, 2, getDeviceName(), "SWITCH_2", RelayLabelsT[1].text, OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

	IUFillNumber(&PWM1N[0], "PWMout1", "%", "%0.0f", 0, 100, 10, 0); 
	IUFillNumberVector(&PWM1NP, PWM1N, 1, getDeviceName(), "PWMOUT1", RelayLabelsT[2].text, OUTPUTS_TAB, IP_RW, 60, IPS_IDLE);	

	IUFillNumber(&PWM2N[0], "PWMout2", "%", "%0.0f", 0, 100, 10, 0); 
	IUFillNumberVector(&PWM2NP, PWM2N, 1, getDeviceName(), "PWMOUT2", RelayLabelsT[3].text, OUTPUTS_TAB, IP_RW, 60, IPS_IDLE);	

	// Set initial relays states to OFF
	for (int i=0; i < 2; i++) relayState[i] = pwmState[i] = 0;    

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

	return true;
}

bool AstroLink4Pi::updateProperties()
{
	INDI::DefaultDevice::updateProperties();

	if (isConnected())
	{
        FI::updateProperties();
        defineProperty(&FocusStepDelayNP);
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

        IDSnoopDevice(ActiveTelescopeT[0].text, "TELESCOPE_INFO");

        if (readDS18B20())
		{
			defineProperty(&FocusTemperatureNP);
			defineProperty(&TemperatureCoefNP);
			defineProperty(&TemperatureCompensateSP);
			readDS18B20(); // update immediately
			lastTemperature = FocusTemperatureN[0].value; // init last temperature
        }
	} else {
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
        FI::updateProperties();
	}

	return true;
}

bool AstroLink4Pi::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	// first we check if it's for our device
	if(!strcmp(dev,getDeviceName()))
	{
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

		// handle temperature coefficient
		if (!strcmp(name, TemperatureCoefNP.name))
		{
			IUUpdateNumber(&TemperatureCoefNP,values,names,n);
			TemperatureCoefNP.s=IPS_OK;
			IDSetNumber(&TemperatureCoefNP, nullptr);
			DEBUGF(INDI::Logger::DBG_SESSION, "Temperature coefficient set to %0.1f steps/°C", TemperatureCoefN[0].value);
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

		// handle PWMouts
		if (!strcmp(name, PWM1NP.name))
		{
			IUUpdateNumber(&PWM1NP,values,names,n);
			PWM1NP.s=IPS_OK;
			IDSetNumber(&PWM1NP, nullptr);
			pwmState[0] = PWM1N[0].value;
			DEBUGF(INDI::Logger::DBG_SESSION, "PWM 1 set to %0.0f", PWM1N[0].value);
			return true;
		}

		if (!strcmp(name, PWM2NP.name))
		{
			IUUpdateNumber(&PWM2NP,values,names,n);
			PWM2NP.s=IPS_OK;
			IDSetNumber(&PWM2NP, nullptr);
			pwmState[1] = PWM2N[0].value;
			DEBUGF(INDI::Logger::DBG_SESSION, "PWM 2 set to %0.0f", PWM2N[0].value);
			return true;
		}

		// handle PWMcycle
		if (!strcmp(name, PWMcycleNP.name))
		{
			IUUpdateNumber(&PWMcycleNP,values,names,n);
			PWMcycleNP.s=IPS_OK;
			IDSetNumber(&PWMcycleNP, nullptr);
			pwmCycleTime = PWMcycleN[0].value;
			DEBUGF(INDI::Logger::DBG_SESSION, "PWM cycle time set to %0.0f ms", PWMcycleN[0].value);
			return true;
		}	        

        if (strstr(name, "FOCUS_"))
            return FI::processNumber(dev, name, values, names, n);        
	}

	return INDI::DefaultDevice::ISNewNumber(dev,name,values,names,n);
}

bool AstroLink4Pi::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
    int rv;

	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
	    // handle temperature compensation
	    if(!strcmp(name, TemperatureCompensateSP.name))
	    {
			IUUpdateSwitch(&TemperatureCompensateSP, states, names, n);

			if ( TemperatureCompensateS[0].s == ISS_ON)
			{
				TemperatureCompensateSP.s = IPS_OK;
				DEBUG(INDI::Logger::DBG_SESSION, "Temperature compensation ENABLED.");
			}

			if ( TemperatureCompensateS[1].s == ISS_ON)
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

			if ( SysControlS[0].s == ISS_ON )
			{
				DEBUG(INDI::Logger::DBG_SESSION, "Astroberry device is set to REBOOT. Confirm or Cancel operation.");
				SysControlSP.s = IPS_BUSY;
				IDSetSwitch(&SysControlSP, NULL);
				
				// confirm switch
				defineProperty(&SysOpConfirmSP);

				return true;
			}
			if ( SysControlS[1].s == ISS_ON )
			{
				DEBUG(INDI::Logger::DBG_SESSION, "Astroberry device is set to SHUT DOWN. Confirm or Cancel operation.");
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

		// handle relay 1
		if (!strcmp(name, Switch1SP.name))
		{
			IUUpdateSwitch(&Switch1SP, states, names, n);

			if ( Switch1S[0].s == ISS_ON )
			{
                millis();
				rv = gpiod_line_set_value(gpio_out1, 1);
				if (rv != 0)
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting Astroberry Relay #1");
					Switch1SP.s = IPS_ALERT;
					Switch1S[0].s = ISS_OFF;
					IDSetSwitch(&Switch1SP, NULL);
					return false;
				}
				relayState[0] =  1;
				DEBUG(INDI::Logger::DBG_SESSION, "Astroberry Relays #1 set to ON");
				Switch1SP.s = IPS_OK;
				Switch1S[1].s = ISS_OFF;
				IDSetSwitch(&Switch1SP, NULL);
				return true;
			}
			if ( Switch1S[1].s == ISS_ON )
			{
				rv = gpiod_line_set_value(gpio_out1, 0);
				if (rv != 0)
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting Astroberry Relay #1");
					Switch1SP.s = IPS_ALERT;
					Switch1S[1].s = ISS_OFF;
					IDSetSwitch(&Switch1SP, NULL);
					return false;
				}
				relayState[0] =  0;
				DEBUG(INDI::Logger::DBG_SESSION, "Astroberry Relays #1 set to OFF");
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

			if ( Switch2S[0].s == ISS_ON )
			{
				rv = gpiod_line_set_value(gpio_out2, 1);
				if (rv != 0)
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting Astroberry Relay #2");
					Switch2SP.s = IPS_ALERT;
					Switch2S[0].s = ISS_OFF;
					IDSetSwitch(&Switch2SP, NULL);
					return false;
				}
				relayState[1] =  1;
				DEBUG(INDI::Logger::DBG_SESSION, "Astroberry Relays #2 set to ON");
				Switch2SP.s = IPS_OK;
				Switch2S[1].s = ISS_OFF;
				IDSetSwitch(&Switch2SP, NULL);
				return true;
			}
			if ( Switch2S[1].s == ISS_ON )
			{
				rv = gpiod_line_set_value(gpio_out2, 0);
				if (rv != 0)
				{
					DEBUG(INDI::Logger::DBG_ERROR, "Error setting Astroberry Relay #2");
					Switch2SP.s = IPS_ALERT;
					Switch2S[1].s = ISS_OFF;
					IDSetSwitch(&Switch2SP, NULL);
					return false;
				}
				relayState[1] =  0;
				DEBUG(INDI::Logger::DBG_SESSION, "Astroberry Relays #2 set to OFF");
				Switch2SP.s = IPS_IDLE;
				Switch2S[0].s = ISS_OFF;
				IDSetSwitch(&Switch2SP, NULL);
				return true;
			}
		}   

		// handle focus motor hold
		if(!strcmp(name, FocusHoldSP.name))
		{
			IUUpdateSwitch(&FocusHoldSP, states, names, n);

			if ( FocusHoldS[0].s == ISS_ON )
				holdPower = 0;

			if ( FocusHoldS[1].s == ISS_ON )
				holdPower = 1;

			if ( FocusHoldS[2].s == ISS_ON )
				holdPower = 2;

			FocusHoldSP.s = IPS_OK;
			IDSetSwitch(&FocusHoldSP, nullptr);
			stepperStandby(true);
			return true;
		}     

		// handle focus resolution
		if(!strcmp(name, FocusResolutionSP.name))
		{
			int last_resolution = resolution;

			IUUpdateSwitch(&FocusResolutionSP, states, names, n);

			//Resolution 1/1
			if ( FocusResolutionS[0].s == ISS_ON )
				resolution = 1;

			//Resolution 1/2
			if ( FocusResolutionS[1].s == ISS_ON )
				resolution = 2;

			//Resolution 1/4
			if ( FocusResolutionS[2].s == ISS_ON )
				resolution = 4;

			//Resolution 1/8
			if ( FocusResolutionS[3].s == ISS_ON )
				resolution = 8;

			//Resolution 1/16
			if ( FocusResolutionS[4].s == ISS_ON )
				resolution = 16;

			//Resolution 1/32
			if ( FocusResolutionS[5].s == ISS_ON )
				resolution = 32;

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

			getFocuserInfo();

			FocusResolutionSP.s = IPS_OK;
			IDSetSwitch(&FocusResolutionSP, nullptr);
			return true;
		}


        if (strstr(name, "FOCUS"))
            return FI::processSwitch(dev, name, states, names, n);        
	}

	return INDI::DefaultDevice::ISNewSwitch(dev,name,states,names,n);
}

bool AstroLink4Pi::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
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

		// handle relay labels
		if (!strcmp(name, RelayLabelsTP.name))
		{
			if (isConnected())
			{
				DEBUG(INDI::Logger::DBG_WARNING, "Cannot set labels while device is connected.");
				return false;
			}

			IUUpdateText(&RelayLabelsTP, texts, names, n);
			RelayLabelsTP.s=IPS_OK;
			IDSetText(&RelayLabelsTP, nullptr);
			DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi labels set . You need to save configuration and restart driver to activate the changes.");
			DEBUGF(INDI::Logger::DBG_DEBUG, "AstroLink 4 Pi labels set to OUT1: %s, OUT2: %s, PWM1: %s, PWM2: %s", RelayLabelsT[0].text, RelayLabelsT[1].text, RelayLabelsT[2].text, RelayLabelsT[3].text);

			return true;
		}        
	}

	return INDI::DefaultDevice::ISNewText(dev,name,texts,names,n);
}

bool AstroLink4Pi::ISSnoopDevice (XMLEle *root)
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

	return true;
}

void AstroLink4Pi::TimerHit()
{
    if(!isConnected())
        return;

    long int timeMillis = millis();
    
	if(backlashTicksRemaining <= 0 && ticksRemaining <= 0)
	{
        if(FocusAbsPosNP.s != IPS_OK)
        {
            // All movement completed/aborted, but still the movement not completed
            // save position to file
            savePosition((int) FocusAbsPosN[0].value * MAX_RESOLUTION / resolution); // always save at MAX_RESOLUTION

            // update abspos value and status
            DEBUGF(INDI::Logger::DBG_SESSION, "Focuser at the position %0.0f.", FocusAbsPosN[0].value);

            FocusAbsPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);

            lastTemperature = FocusTemperatureN[0].value; // register last temperature
            stepperStandby(true);
        }
        else
        {
            // Do other stuff only while the stepper is not moving
			if(nextTemperatureRead < timeMillis) 
            {
                readDS18B20();
                nextTemperatureRead = timeMillis + TEMPERATURE_UPDATE_TIMEOUT;
            }
            if(nextTemperatureCompensation < timeMillis)
            {
                temperatureCompensation();
                nextTemperatureCompensation = timeMillis + TEMPERATURE_COMPENSATION_TIMEOUT;
            }
            if(nextSystemRead < timeMillis)
            {
                systemUpdate();
                nextSystemRead = timeMillis + SYSTEM_UPDATE_PERIOD;
            }    
            if(nextPwmCycle < timeMillis)
            {
                pwmCycle();
                nextPwmCycle = timeMillis + PWM_CYCLE_PERIOD;
            }     
        }
	} 
    else
    {
        // Progress with movement
        int motorDirection = lastDirection;
		
        // handle Reverse Motion
        if (FocusReverseS[INDI_ENABLED].s == ISS_ON) 
        {
            motorDirection = -1 * motorDirection;
        }

        bool isBacklash = (backlashTicksRemaining > 0);
 
        //Move the actual motor
        stepMotor(motorDirection);

        if(isBacklash == false)
        {   //Only Count the position change if it is not due to backlash
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
        {   //Don't count the backlash position change, just decrement the counter
            backlashTicksRemaining -= 1;
        }
    }
	SetTimer(FocusStepDelayN[0].value);
}

bool AstroLink4Pi::AbortFocuser()
{
	DEBUG(INDI::Logger::DBG_SESSION, "Focuser motion aborted.");
	backlashTicksRemaining = 0;
	ticksRemaining = 0;
	return true;
}

void AstroLink4Pi::stepMotor(int direction)
{
	if (direction < 0) gpiod_line_set_value(gpio_dir, 0); else gpiod_line_set_value(gpio_dir, 1);
	// step on
	gpiod_line_set_value(gpio_stp, 1);
	// wait
	usleep(10);
	// step off
	gpiod_line_set_value(gpio_stp, 0);
}

IPState AstroLink4Pi::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
	uint32_t targetTicks = FocusAbsPosN[0].value + ((int32_t)ticks * (dir == FOCUS_INWARD ? -1 : 1));
	return MoveAbsFocuser(targetTicks);
}

IPState AstroLink4Pi::MoveAbsFocuser(uint32_t targetTicks)
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
	stepperStandby(false);

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

	// if direction changed do backlash adjustment
	if ( lastDirection != 0 && newDirection != lastDirection && FocusBacklashN[0].value != 0)
	{
		DEBUGF(INDI::Logger::DBG_SESSION, "Backlash compensation by %0.0f steps.", FocusBacklashN[0].value);
		backlashTicksRemaining = FocusBacklashN[0].value;
	}
	else
	{
		backlashTicksRemaining = 0;
	}

	lastDirection = newDirection;

	// process targetTicks
	ticksRemaining = abs(targetTicks - FocusAbsPosN[0].value);

	DEBUGF(INDI::Logger::DBG_SESSION, "Focuser is moving %s to position %d.", direction, targetTicks);

	SetTimer(FocusStepDelayN[0].value);

	return IPS_BUSY;
}

void AstroLink4Pi::SetResolution(int res)
{
	// Release lines
	gpiod_line_set_value(gpio_m0, 1);
	gpiod_line_set_value(gpio_m1, 1);
	gpiod_line_set_value(gpio_m2, 1);

	switch(res)
	{
		case 1:	// 1:1
			gpiod_line_set_value(gpio_m0, 0);
			gpiod_line_set_value(gpio_m1, 0);
			gpiod_line_set_value(gpio_m2, 0);
			break;
		case 2:	// 1:2
			gpiod_line_set_value(gpio_m0, 1);
			gpiod_line_set_value(gpio_m1, 0);
			gpiod_line_set_value(gpio_m2, 0);
			break;
		case 4:	// 1:4
			gpiod_line_set_value(gpio_m0, 0);
			gpiod_line_set_value(gpio_m1, 1);
			gpiod_line_set_value(gpio_m2, 0);
			break;
		case 8:	// 1:8
			gpiod_line_set_value(gpio_m0, 0);
			gpiod_line_set_value(gpio_m1, 1);
			gpiod_line_set_value(gpio_m2, 0);
			break;
		case 16:	// 1:16
			gpiod_line_set_value(gpio_m0, 0);
			gpiod_line_set_value(gpio_m1, 0);
			gpiod_line_set_value(gpio_m2, 1);
			break;
		case 32:	// 1:32
			gpiod_line_set_value(gpio_m0, 1);
			gpiod_line_set_value(gpio_m1, 1);
			gpiod_line_set_value(gpio_m2, 1);
			break;
		default:	// 1:1
			gpiod_line_set_value(gpio_m0, 0);
			gpiod_line_set_value(gpio_m1, 0);
			gpiod_line_set_value(gpio_m2, 0);

			break;
	}


	DEBUGF(INDI::Logger::DBG_SESSION, "Resolution set to 1 / %d.", res);
}

bool AstroLink4Pi::ReverseFocuser(bool enabled)
{
	if (enabled)
	{
		DEBUG(INDI::Logger::DBG_SESSION, "Reverse direction ENABLED.");
	} else {
		DEBUG(INDI::Logger::DBG_SESSION, "Reverse direction DISABLED.");
	}
	return true;
}

int AstroLink4Pi::savePosition(int pos)
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

void AstroLink4Pi::temperatureCompensation()
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
}

bool AstroLink4Pi::readDS18B20()
{
    if (!isConnected())
		return false;

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

void AstroLink4Pi::stepperStandby(bool disabled)
{
	if (!isConnected())
		return;
 
    if (holdPower == 2 || !disabled) 
	{
		gpiod_line_set_value(gpio_en, 0);
		gpiod_line_set_value(gpio_hold, 0);
		DEBUG(INDI::Logger::DBG_SESSION, "Stepper motor enabled 100%%.");
	}
	else if (holdPower == 1 || !disabled) 
	{
		gpiod_line_set_value(gpio_en, 0);
		gpiod_line_set_value(gpio_hold, 1);
		DEBUG(INDI::Logger::DBG_SESSION, "Stepper motor enabled 50%%.");
	}
	else
	{
		gpiod_line_set_value(gpio_en, 1);
		gpiod_line_set_value(gpio_hold, 1);
		DEBUG(INDI::Logger::DBG_SESSION, "Stepper motor disabled.");
	}
}

void AstroLink4Pi::systemUpdate()
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
}

void AstroLink4Pi::getFocuserInfo()
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

long int AstroLink4Pi::millis()
{    
    struct timespec clock;
    if(clock_gettime(CLOCK_MONOTONIC, &clock) == 0)
    {
       static long int tv_sec_zero = clock.tv_sec;
       return 1000 * (clock.tv_sec - tv_sec_zero) + clock.tv_nsec / 1000000;
    }    
    else
    {
        DEBUG(INDI::Logger::DBG_ERROR, "CLOCK_MONOTONIC not available.");
        return 0;
    }    
}

void AstroLink4Pi::pwmCycle()
{
	if (!isConnected())
		return;

	pwmCounter++;
	if(pwmCounter > 10) pwmCounter = 0;
	gpiod_line_set_value(gpio_pwm1, pwmState[0] > 10*pwmCounter);
	gpiod_line_set_value(gpio_pwm2, pwmState[1] > 10*((pwmCounter + 5) % 10));
}

