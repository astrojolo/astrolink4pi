
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
	virtual bool ISSnoopDevice(XMLEle *root);

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
	virtual bool readDS18B20();

	ISwitch FocusResolutionS[6];
	ISwitchVectorProperty FocusResolutionSP;
	ISwitch FocusHoldS[3];
	ISwitchVectorProperty FocusHoldSP;
	INumber FocusStepDelayN[1];
	INumberVectorProperty FocusStepDelayNP;
	INumber FocusTemperatureN[1];
	INumberVectorProperty FocusTemperatureNP;
	INumber TemperatureCoefN[1];
	INumberVectorProperty TemperatureCoefNP;
	ISwitch TemperatureCompensateS[2];
	ISwitchVectorProperty TemperatureCompensateSP;	

	INumber FocuserInfoN[3];
	INumberVectorProperty FocuserInfoNP;
	INumber FocuserTravelN[1];
	INumberVectorProperty FocuserTravelNP;	

	INumber ScopeParametersN[2];
	INumberVectorProperty ScopeParametersNP;
	IText ActiveTelescopeT[1];
	ITextVectorProperty ActiveTelescopeTP;	

	IText SysTimeT[2];
	ITextVectorProperty SysTimeTP;
	IText SysInfoT[7];
	ITextVectorProperty SysInfoTP;
	ISwitch SysControlS[2];
	ISwitchVectorProperty SysControlSP;
	ISwitch SysOpConfirmS[2];
	ISwitchVectorProperty SysOpConfirmSP;	

	IText RelayLabelsT[4];
	ITextVectorProperty RelayLabelsTP;

	ISwitch Switch1S[2];
	ISwitchVectorProperty Switch1SP;
	ISwitch Switch2S[2];
	ISwitchVectorProperty Switch2SP;

	INumber PWM1N[1];
	INumberVectorProperty PWM1NP;
	INumber PWM2N[1];
	INumberVectorProperty PWM2NP;

	INumber PWMcycleN[1];
	INumberVectorProperty PWMcycleNP;	


	// struct gpiod_chip *chip;
	// struct gpiod_line *gpio_en;
	// struct gpiod_line *gpio_m0;
	// struct gpiod_line *gpio_m1;
	// struct gpiod_line *gpio_m2;
	// struct gpiod_line *gpio_rst;
	// struct gpiod_line *gpio_stp;
	// struct gpiod_line *gpio_dir;
	// struct gpiod_line *gpio_out1;
	// struct gpiod_line *gpio_out2;
	// struct gpiod_line *gpio_pwm1;
	// struct gpiod_line *gpio_pwm2;	
	// struct gpiod_line *gpio_hold;	

	int pigpioHandle = -1;
	int resolution = 1;
	int holdPower = 0;
	float lastTemperature;

	int backlashTicksRemaining;
	int ticksRemaining;
	int lastDirection = 0;
	int currentStep = -1;
	bool abortStep = false;		

	int pwmState[2];
	int relayState[2];
	int pwmCounter = 0;	
	int pwmCycleTime = 100;

	long int nextTemperatureRead = 0;
	long int nextTemperatureCompensation = 0;
	long int nextSystemRead = 0;
	long int nextPwmCycle = 0;

	void getFocuserInfo();
	void temperatureCompensation();
	void stepperStandby(bool state);
	void systemUpdate();
	void pwmCycle();
	int setDac(int value);
	long int millis();

	static constexpr const char *SYSTEM_TAB {"System"};
	static constexpr const char *OUTPUTS_TAB {"Outputs"};
};

#endif
