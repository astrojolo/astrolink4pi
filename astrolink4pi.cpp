
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

#include "astroberry4pi.h"

//////////////////////////////////////////////////////////////////////
/// Delegates
//////////////////////////////////////////////////////////////////////
std::unique_ptr<IndiAstrolink4Pi> indiIndiAstrolink4Pi(new IndiAstrolink4Pi());

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

IndiAstrolink4Pi::IndiAstrolink4Pi()
{
	setVersion(VERSION_MAJOR,VERSION_MINOR);
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
		
		// average 15 measurements, so the fan will not overreact
/*      cpuTemps[cpuTempsIndex] = atoi(buffer);
		cpuTempsIndex++;
		if(cpuTempsIndex > 14) cpuTempsIndex = 0;
		int sum = 0;
		for(int i = 0; i < 15; i++) sum += cpuTemps[i];

		int newFanPWM = ((sum / 15 - FanTempN[0].value) / 2);
		if(newFanPWM < 0) newFanPWM = 0;
		if(newFanPWM > 10) newFanPWM = 10;
		if(newFanPWM != fanPWM)
		{
			fanPWM = newFanPWM;
			snprintf(ts, sizeof(ts), "%4.0f", (10.0*fanPWM));
			IUSaveText(&SysInfoT[7], ts);			
		}
*/
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
	if (gpiod_line_is_used(gpiod_chip_get_line(chip, PIN_SYS_FAN)))
	{
		DEBUGF(INDI::Logger::DBG_ERROR, "BCM Pin %0.0f already used", PIN_SYS_FAN);
		gpiod_chip_close(chip);
		return false;
	}

	// Select gpios
	gpio_sysfan = gpiod_chip_get_line(chip, PIN_SYS_FAN);
	// Set initial gpios direction and states
	gpiod_line_request_output(gpio_sysfan, "astroberry_sysfan", 0);

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

    return true;
}

bool IndiAstrolink4Pi::updateProperties()
{
	// Call parent update properties first
	INDI::DefaultDevice::updateProperties();

	if (isConnected())
	{
		defineText(&SysTimeTP);
		defineText(&SysInfoTP);
		defineSwitch(&SysControlSP);
	}
	else
	{
		// We're disconnected
		deleteProperty(SysTimeTP.name);
		deleteProperty(SysInfoTP.name);
		deleteProperty(SysControlSP.name);
	}
	return true;
}

void IndiAstrolink4Pi::ISGetProperties(const char *dev)
{
	INDI::DefaultDevice::ISGetProperties(dev);
}

bool IndiAstrolink4Pi::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	return INDI::DefaultDevice::ISNewNumber(dev,name,values,names,n);
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
				defineSwitch(&SysOpConfirmSP);

				return true;
			}
			if ( SysControlS[1].s == ISS_ON )
			{
				DEBUG(INDI::Logger::DBG_SESSION, "AstroLink 4 Pi device is set to SHUT DOWN. Confirm or Cancel operation.");
				SysControlSP.s = IPS_BUSY;
				IDSetSwitch(&SysControlSP, NULL);

				// confirm switch
				defineSwitch(&SysOpConfirmSP);

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
	}
	return INDI::DefaultDevice::ISNewSwitch (dev, name, states, names, n);
}

bool IndiAstrolink4Pi::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	return INDI::DefaultDevice::ISNewText (dev, name, texts, names, n);
}

bool IndiAstrolink4Pi::ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
	return INDI::DefaultDevice::ISNewBLOB (dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool IndiAstrolink4Pi::saveConfigItems(FILE *fp)
{
	//IUSaveConfigNumber(fp, &FanTempNP);

	return true;
}