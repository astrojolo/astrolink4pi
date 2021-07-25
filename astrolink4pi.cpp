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
	FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_REVERSE | FOCUSER_CAN_SYNC | FOCUSER_CAN_ABORT); 
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
	DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi connected successfully.");

	return true;
}

bool AstroLink4Pi::Disconnect()
{
	DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi disconnected successfully.");

	return true;
}

bool AstroLink4Pi::initProperties()
{
	INDI::DefaultDevice::initProperties();
    FI::initProperties(OPTIONS_TAB);

	// Add default properties
	// addAuxControls(); // use instead if simulation mode is added to code
	addDebugControl ();
	addConfigurationControl();

	return true;
}

bool AstroLink4Pi::updateProperties()
{
	INDI::DefaultDevice::updateProperties();

	if (isConnected())
	{
        FI::updateProperties();
	} else {
        FI::updateProperties();
	}

	return true;
}

bool AstroLink4Pi::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	// first we check if it's for our device
	if(!strcmp(dev,getDeviceName()))
	{
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
	return true;
}
