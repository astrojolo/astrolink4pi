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


class IndiAstrolink4Pi : public INDI::DefaultDevice
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

protected:
	virtual bool saveConfigItems(FILE *fp);
	virtual void TimerHit();
private:
	virtual bool Connect();
	virtual bool Disconnect();

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

	int fanPWM = 0;			// 0 to 10
	int fanCycle = 0;
	int cpuTemps[15];
	int cpuTempsIndex = 0;

	const char* gpio_chip_path = "/dev/gpiochip0";
	struct gpiod_chip *chip;
	struct gpiod_line *gpio_sysfan;

	int fanControlID { -1 };
	void fanControl();    

};

#endif