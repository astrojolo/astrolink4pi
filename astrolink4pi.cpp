/**
 * Updated code following improvements: 
 *  Fix: savePosition close on failure; correct ADC 16-bit reads; use atomic for thread abort; use sleep_for; snprintf
 */

#include "astrolink4pi.h"

std::unique_ptr<AstroLink4Pi> astroLink4Pi(new AstroLink4Pi());

#define ACS_TYPE 0		// 0 - 20A, 1 - 5A

#define MAX_RESOLUTION 32							 // the highest resolution supported is 1/32 step
#define TEMPERATURE_UPDATE_TIMEOUT (5 * 1000)   		 // 5 sec
#define TEMPERATURE_COMPENSATION_TIMEOUT (30 * 1000) // 30 sec
#define SYSTEM_UPDATE_PERIOD 1000
#define POLL_PERIOD 200
#define FAN_PERIOD (20 * 1000)

#define TSL2591_ADC_TIME 750  // integration time in ms for a single increment
#define TSL2591_ADDR (0x29)
#define TSL2591_COMMAND_BIT (0xA0)  // bits 7 and 5 for 'command normal'
#define TSL2591_ENABLE_POWERON (0x01)
#define TSL2591_ENABLE_POWEROFF (0x00)
#define TSL2591_ENABLE_AEN (0x02)
#define TSL2591_ENABLE_AIEN (0x10)
#define TSL2591_REGISTER_ENABLE 0x00
#define TSL2591_REGISTER_CONTROL 0x01
#define TSL2591_REGISTER_CHAN0_LOW 0x14
#define TSL2591_REGISTER_CHAN1_LOW 0x16
#define FILTER_COEFF -1.2

#define RP4_GPIO 0
#define RP5_GPIO 4
#define DECAY_PIN 14
#define EN_PIN 15
#define M0_PIN 17
#define M1_PIN 18
#define M2_PIN 27
#define RST_PIN 22
#define STP_PIN 24
#define DIR_PIN 23
#define OUT1_PIN 5
#define OUT2_PIN 6
#define PWM1_PIN 26
#define PWM2_PIN 19
#define MOTOR_PWM 20
#define CHK_IN_PIN 16
#define FAN_PIN 13

void ISPoll(void *p);

void ISInit() { ... }

/** 
 * Set the resolution based on the provided parameter
 * @param res The resolution to set
 */
void AstroLink4Pi::SetResolution(int res) { ... }

/**
 * Move the focuser to an absolute position
 */
IPState AstroLink4Pi::MoveAbsFocuser(uint32_t targetTicks) { ... }

/** 
 * Save the current position of focuser to a file.
 */
int AstroLink4Pi::savePosition(int pos) { ... }

