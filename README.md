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

# Source
https://github.com/astrojolo/astrolink4pi

# Requirements
* INDI available here http://indilib.org/download.html
* CMake >= 2.4.7

# Installation
Download and install required libraries before compiling AstroLink 4 Pi. See [INDI site](http://indilib.org/download.html) for more details.
In most cases it's enough to run:
```
sudo apt-get install cmake libindi-dev libgpiod-dev
```
Then you can compile the driver:
```
git clone https://github.com/astrojolo/astrolink4pi
cd astrolink4pi
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
```
You can install the drivers by running:
```
sudo make install
```
OR manually installing files by running:
```
sudo copy indi_astrolink4pi /usr/bin/
sudo copy indi_astrolink4pi.xml /usr/share/indi/
```

# How to use it?
Enable 1-Wire interface using raspi-config or adding 'dtoverlay=w1-gpio' to /boot/configure.txt for temperature compensation support (reboot required). Run Kstars and select AstroLink 4 Pi (Aux section) in Ekos profile editor. Then start INDI server in Ekos with your profile, containg AstroLink 4 Pi drivers. Alternatively you can start INDI server manually by running:
```
indi_server indi_astrolink4pi
```
Start KStars with Ekos, connect to your INDI server and enjoy!

Note that your user account needs proper access right to /dev/gpiochip0 device. By default you can read/write only if you run driver as root or user who is a member of gpio group. Add your user to gpio group by running ```sudo usermod -a -G gpio $USER```

To use restart/shutdown functionality add this line to your /etc/sudoers file or /etc/sudoers.d/010_astroberry-nopasswd (this assumes you run INDI server as astroberry user):
```
astroberry ALL=(ALL) NOPASSWD: /sbin/reboot, /sbin/poweroff
```

For custom labels you need to save configuration and restart the driver after changing relays' labels.

# AstroLink 4 Pi micro images

![Schematic](/images/astrolink4pi-micro.png)

![PCB](/images/astrolink4pi-micro-pcb.png)

![Photo](/images/astrolink4pi-micro.jpg)
