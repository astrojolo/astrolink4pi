
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


#include <defaultdevice.h>
#include <indifocuserinterface.h>


class AstroLink4Pi : public INDI::DefaultDevice, public INDI::FocuserInterface
{
public:
	AstroLink4Pi();
	virtual ~AstroLink4Pi();

	virtual bool initProperties();
	virtual bool updateProperties();

	virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
	virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
	virtual bool ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);

protected:
	const char *getDefaultName();

	// Focuser Overrides
    virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
	virtual bool ReverseFocuser(bool enabled);
	virtual bool AbortFocuser();
	virtual bool SyncFocuser(uint32_t ticks) override;
	virtual bool SetFocuserBacklash(int32_t steps) override;	

	virtual bool saveConfigItems(FILE *fp);
	virtual void TimerHit();

private:
	virtual bool Connect();
	virtual bool Disconnect();
	virtual void SetResolution(int res);
	virtual int savePosition(int pos);
	virtual void stepMotor(int direction);

	INumber FocusStepDelayN[1];
	INumberVectorProperty FocusStepDelayNP;

	struct gpiod_chip *chip;
	struct gpiod_line *gpio_a1;
	struct gpiod_line *gpio_a2;
	struct gpiod_line *gpio_b1;
	struct gpiod_line *gpio_b2;

	int resolution = 1;
	float lastTemperature;

	int backlashTicksRemaining;
	int ticksRemaining;
	int lastDirection = 0;
	int currentStep = -1;
	bool abortStep = false;		

	static constexpr const char *SETTINGS_TAB {"Settings"};
};

#endif
