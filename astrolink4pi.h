
#ifndef FOCUSRPI_H
#define FOCUSRPI_H

#include <indifocuser.h>

class AstroberryFocuser : public INDI::Focuser
{
public:
	AstroberryFocuser();
	virtual ~AstroberryFocuser();
	const char *getDefaultName();
	virtual bool initProperties();
	virtual bool updateProperties();
	virtual void ISGetProperties (const char *dev);
	virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
	virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
	virtual bool ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);
	virtual bool ISSnoopDevice(XMLEle *root);
	static void stepperStandbyHelper(void *context);
	static void updateTemperatureHelper(void *context);
	static void temperatureCompensationHelper(void *context);
protected:
	virtual IPState MoveAbsFocuser(int ticks);
	virtual IPState MoveRelFocuser(FocusDirection dir, int ticks);
	virtual bool saveConfigItems(FILE *fp);
	virtual bool ReverseFocuser(bool enabled);
	virtual bool AbortFocuser();
	virtual void TimerHit();
	virtual bool SyncFocuser(uint32_t ticks) override;
private:
	virtual bool Connect();
	virtual bool Disconnect();
	virtual void SetResolution(int res);
	virtual int savePosition(int pos);
	virtual bool readDS18B20();
	virtual void stepMotor(int direction);

	ISwitch MotorDirS[2];
	ISwitchVectorProperty MotorDirSP;
	ISwitch FocusResolutionS[2];
	ISwitchVectorProperty FocusResolutionSP;
	INumber FocuserInfoN[3];
	INumberVectorProperty FocuserInfoNP;
	INumber FocusStepDelayN[1];
	INumberVectorProperty FocusStepDelayNP;
	INumber FocusBacklashN[1];
	INumberVectorProperty FocusBacklashNP;
	INumber FocuserTravelN[1];
	INumberVectorProperty FocuserTravelNP;
	ISwitch ResetAbsPosS[1];
	ISwitchVectorProperty ResetAbsPosSP;
	IText ActiveTelescopeT[1];
	ITextVectorProperty ActiveTelescopeTP;
	INumber ScopeParametersN[2];
	INumberVectorProperty ScopeParametersNP;
	INumber FocusTemperatureN[1];
	INumberVectorProperty FocusTemperatureNP;
	INumber TemperatureCoefN[1];
	INumberVectorProperty TemperatureCoefNP;
	ISwitch TemperatureCompensateS[2];
	ISwitchVectorProperty TemperatureCompensateSP;

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

	void getFocuserInfo();
	int stepperStandbyID { -1 };
	void stepperStandby();
	int updateTemperatureID { -1 };
	void updateTemperature();
	int temperatureCompensationID { -1 };
	void temperatureCompensation();
};

#endif
