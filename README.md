# AstroLink 4 Pi

AstroLink 4 Pi micro features:
* Focuser
  - ULN2003 driver support for Moonlite / Robofocus / AstroLink geared unipolar steppers
  - Direct stepper motor control without proprietary drivers
  - Absolute position control
  - Relative position control
  - Forward / Reverse direction configuration
  - Customizable maximum absolute position (steps)
  - Customizable maximum focuser travel (mm)
  - Resolution control full step and half step
  - Backlash compensation
  - Speed control
  - Focuser info including: critical focus zone in μm, step size in μm, steps per critical focus zone
  - Automatic temperature compensation based on DS18B20 temperature sensor
  - Stepper movement abort
* Power outputs
  - Two switchable 12V DC outputs, 5A max each
  - Two PWM regulated RCA outputs, 3A max each
  - Configurable labels
* Astroberry System
  - Provides system information such as local system time, UTC offset, hardware identification, CPU temperature, uptime, system load, hostname, local IP, public IP
  - Allows for system restart and shut down (Supported on linux operating system only. Requires advanced configuration of sudo to allow restart & shutdown without password)
  - System fan control with speed controlled by CPU temperature
* Other
  - One output 5V
  - One adjustable voltage output 3-10V
  - One permanent 12V DC output