#pragma once

// ── HAL wrappers ─────────────────────────────────────────────────────────────
#include "hal/GpioPin.h"
#include "hal/UartBus.h"
#include "hal/I2cBus.h"
#include "hal/PwmOutput.h"

// ── IC drivers ───────────────────────────────────────────────────────────────
#include "drivers/TMC2209.h"
#include "drivers/SHT30.h"
#include "drivers/MLX90641.h"
#include "drivers/TSL2561.h"
#include "drivers/INA219.h"
#include "drivers/TPS1663.h"
#include "drivers/AT24Cxx.h"
#include "drivers/Rtc.h"

// ── Services ─────────────────────────────────────────────────────────────────
#include "services/MotorService.h"
#include "services/SensorService.h"
#include "services/PowerService.h"
#include "services/SchedulerService.h"

// ── Protocol ─────────────────────────────────────────────────────────────────
#include "protocol/Dispatcher.h"

// ── Config ───────────────────────────────────────────────────────────────────
#include "config/PersistentConfig.h"

#include "main.h"  // CubeMX-generated HAL handle externs
#include <array>

// Top-level application class.
//
// Owns every object in the system.  main.c creates one instance, passes in
// all CubeMX HAL handles, then calls init() once and run() from while(1).
// The timer ISR calls motorTick() directly.
//
// Object construction order mirrors dependency order:
//   HAL wrappers → drivers → services / protocol / config
//
// All HAL handles (huart1, hi2c1, …) are extern-declared by CubeMX in main.h.
// Passing them explicitly here keeps App testable and avoids global state.

class App
{
public:
    // All CubeMX-generated HAL handles the application needs.
    struct Handles
    {
        UART_HandleTypeDef* huartHost;   // UART to CP2102 / host PC
        UART_HandleTypeDef* huartMotor;  // UART to TMC2209 PDN_UART (single-wire)
        I2C_HandleTypeDef*  hi2c;        // shared I2C bus (all sensors + EEPROM)
        TIM_HandleTypeDef*  htimPwm;     // timer configured for 40 kHz PWM
        RTC_HandleTypeDef*  hrtc;
    };

    explicit App(const Handles& h);

    // Initialises all drivers, loads config, registers protocol handlers.
    // Returns false if a critical peripheral fails (UART, EEPROM).
    bool init();

    // Call from while(1) in main.c.  Non-blocking; processes one protocol
    // frame, one sensor round, and checks power faults.
    void run();

    // Call from the hardware timer ISR (100 kHz recommended).
    void motorTick();

private:
    void registerProtocolHandlers();
    void applyConfigToHardware();

    // ── HAL wrappers (no heap allocation) ─────────────────────────────────────
    hal::UartBus    _hostUart;
    hal::UartBus    _motorUart;
    hal::I2cBus     _i2c;
    hal::PwmOutput  _pwm0;
    hal::PwmOutput  _pwm1;

    // TPS1663 GPIO pins — EN active-high, FAULT active-low
    // Adjust port/pin constants to match your PCB schematic.
    hal::GpioPin _dc0En, _dc0Fault;
    hal::GpioPin _dc1En, _dc1Fault;
    hal::GpioPin _dc2En, _dc2Fault;

    // TMC2209 step/dir/en pins
    hal::GpioPin _motorStep, _motorDir, _motorEn;

    // ── IC drivers ────────────────────────────────────────────────────────────
    drv::TMC2209::Config _motorCfg;
    drv::TMC2209   _motor;
    drv::SHT30     _sht30;
    drv::MLX90641  _mlx;
    drv::TSL2561   _tsl;
    drv::INA219    _ina219;
    drv::TPS1663   _dc0, _dc1, _dc2;
    drv::AT24Cxx   _eeprom;
    drv::Rtc       _rtc;

    // ── Services ──────────────────────────────────────────────────────────────
    svc::MotorService      _motorSvc;
    svc::SensorService     _sensorSvc;
    svc::PowerService      _powerSvc;
    svc::SchedulerService  _schedulerSvc;

    // ── Protocol & config ─────────────────────────────────────────────────────
    proto::Dispatcher      _dispatcher;
    cfg::PersistentConfig  _config;
};
