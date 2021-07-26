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
#include "config.h"


#include <gpiod.h>

#include "astrolink4pi.h"


std::unique_ptr<AstroLink4Pi> astroLink4Pi(new AstroLink4Pi());

#define MAX_RESOLUTION                      2 // the highest resolution supported is 1/2 step
#define TEMPERATURE_UPDATE_TIMEOUT          (3 * 1000) // 3 sec
#define STEPPER_STANDBY_TIMEOUT             (2 * 1000) // 2 sec
#define TEMPERATURE_COMPENSATION_TIMEOUT    (30 * 1000) // 60 sec

#define A1_PIN	23
#define A2_PIN	24
#define B1_PIN	18
#define B2_PIN	17

int halfStep[8][4] = { {1,0,0,0}, {1,0,1,0}, {0,0,1,0}, {0,1,1,0}, {0,1,0,0}, {0,1,0,1}, {0,0,0,1}, {1,0,0,1} };
int fullStep[8][4] = { {1,0,0,0}, {0,0,1,0}, {0,1,0,0}, {0,0,0,1}, {1,0,0,0}, {0,0,1,0}, {0,1,0,0}, {0,0,0,1} };

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


AstroLink4Pi::AstroLink4Pi() : FI(this)
{
	setVersion(VERSION_MAJOR,VERSION_MINOR);	
}

AstroLink4Pi::~AstroLink4Pi()
{
	// delete properties independent of connection status
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
		DEBUG(INDI::Logger::DBG_ERROR, "Problem initiating Astroberry Focuser.");
		return false;
	}

	// verify BCM Pins are not used by other consumers
	int pins[] = {A1_PIN, A2_PIN, B1_PIN, B2_PIN};
	for (unsigned int pin = 0; pin < 4; pin++)
	{
		if (gpiod_line_is_used(gpiod_chip_get_line(chip, pins[pin])))
		{
			DEBUGF(INDI::Logger::DBG_ERROR, "BCM Pin %0.0f already used", pins[pin]);
			gpiod_chip_close(chip);
			return false;
		}
	}

	// Select gpios
	gpio_a1 = gpiod_chip_get_line(chip, A1_PIN);
	gpio_a2 = gpiod_chip_get_line(chip, A2_PIN);
	gpio_b1 = gpiod_chip_get_line(chip, B1_PIN);
	gpio_b2 = gpiod_chip_get_line(chip, B2_PIN);


	// Set initial state for gpios
	gpiod_line_request_output(gpio_a1, "a1@astroberry_focuser", 0);
	gpiod_line_request_output(gpio_a2, "a2@astroberry_focuser", 0);
	gpiod_line_request_output(gpio_b1, "b1@astroberry_focuser", 0); 
	gpiod_line_request_output(gpio_b2, "b2@astroberry_focuser", 0);

	//read last position from file & convert from MAX_RESOLUTION to current resolution
	FocusAbsPosN[0].value = savePosition(-1) != -1 ? (int) savePosition(-1) * resolution / MAX_RESOLUTION : 0;

	// preset resolution
	// SetResolution(resolution);

	// Update focuser parameters
	// getFocuserInfo();

	// set motor standby timer
	// stepperStandbyID = IEAddTimer(STEPPER_STANDBY_TIMEOUT, stepperStandbyHelper, this);

	DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi connected successfully.");

	return true;
}

bool AstroLink4Pi::Disconnect()
{
	// Close device
	gpiod_chip_close(chip);

	// Stop timers
	// IERmTimer(stepperStandbyID);
	IERmTimer(updateTemperatureID);
	IERmTimer(temperatureCompensationID);
  
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

    addDebugControl();
    addSimulationControl();
    addConfigurationControl();

    // Step delay setting
	IUFillNumber(&FocusStepDelayN[0], "FOCUS_STEPDELAY_VALUE", "milliseconds", "%0.0f", 2, 50, 1, 5);
	IUFillNumberVector(&FocusStepDelayNP, FocusStepDelayN, 1, getDeviceName(), "FOCUS_STEPDELAY", "Step Delay", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

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
        }
	} else {
		deleteProperty(FocusTemperatureNP.name);
		deleteProperty(TemperatureCoefNP.name);
		deleteProperty(TemperatureCompensateSP.name);        
        deleteProperty(FocusStepDelayNP.name);
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
			return true;
		}        

        if (strstr(name, "FOCUS_"))
            return FI::processNumber(dev, name, values, names, n);        
	}

	return INDI::DefaultDevice::ISNewNumber(dev,name,values,names,n);
}

bool AstroLink4Pi::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
	// first we check if it's for our device
	if (!strcmp(dev, getDeviceName()))
	{
        if (strstr(name, "FOCUS"))
            return FI::processSwitch(dev, name, states, names, n);        
	}

	return INDI::DefaultDevice::ISNewSwitch(dev,name,states,names,n);
}

bool AstroLink4Pi::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	return INDI::DefaultDevice::ISNewText(dev,name,texts,names,n);
}

bool AstroLink4Pi::saveConfigItems(FILE *fp)
{
    FI::saveConfigItems(fp);
	// IUSaveConfigSwitch(fp, &FocusResolutionSP);
	// IUSaveConfigSwitch(fp, &FocusReverseSP);
	// IUSaveConfigSwitch(fp, &TemperatureCompensateSP);
	// IUSaveConfigNumber(fp, &FocusMaxPosNP);
	// IUSaveConfigNumber(fp, &FocusStepDelayNP);
	// IUSaveConfigNumber(fp, &FocusBacklashNP);
	// IUSaveConfigNumber(fp, &FocuserTravelNP);
	// IUSaveConfigNumber(fp, &TemperatureCoefNP);

	return true;
}

void AstroLink4Pi::TimerHit()
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
		// lastTemperature = FocusTemperatureN[0].value; // register last temperature

		// set motor standby timer
		// IERmTimer(stepperStandbyID);
		// stepperStandbyID = IEAddTimer(STEPPER_STANDBY_TIMEOUT, stepperStandbyHelper, this);

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

void AstroLink4Pi::SetResolution(int res)
{
	DEBUGF(INDI::Logger::DBG_SESSION, "Resolution set to 1 / %0.0f.", res);
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

void AstroLink4Pi::updateTemperatureHelper(void *context)
{
	static_cast<AstroLink4Pi*>(context)->updateTemperature();
}

void AstroLink4Pi::temperatureCompensationHelper(void *context)
{
	static_cast<AstroLink4Pi*>(context)->temperatureCompensation();
}

void AstroLink4Pi::updateTemperature()
{
	if (!isConnected())
		return;

	readDS18B20();
	updateTemperatureID = IEAddTimer(TEMPERATURE_UPDATE_TIMEOUT, updateTemperatureHelper, this);
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

	temperatureCompensationID = IEAddTimer(TEMPERATURE_COMPENSATION_TIMEOUT, temperatureCompensationHelper, this);
}
