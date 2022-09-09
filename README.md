# AstroLink 4 Pi

AstroLink 4 Pi micro features:
* Focuser
  - DRV8825 driver support for Moonlite / Robofocus / AstroLink geared unipolar steppers and bipolar microstepping up to 1/32
  - Absolute position control
  - Relative position control
  - Forward / Reverse direction configuration
  - Customizable maximum absolute position (steps)
  - Customizable maximum focuser travel (mm)
  - Backlash compensation
  - Speed control
  - Focuser info including: critical focus zone in μm, step size in μm, steps per critical focus zone
  - Automatic temperature compensation based on DS18B20 temperature sensor
  - Stepper movement abort
  - 6 pin RJ12 stepper output
  - embedded real time clock (AstroLink 4 Pi version 2 only)
* Power outputs
  - Two switchable 12V DC outputs, 5A max each
  - One permanent 12V DC output
  - Two PWM regulated RCA outputs, 3A max each
  - One adjustable DC output 3-10V, 1.5A max
  - Configurable labels
* Astroberry System
  - Provides system information such as local system time, UTC offset, hardware identification, CPU temperature, uptime, system load, hostname, local IP, public IP
  - Allows for system restart and shut down (Supported on linux operating system only. Requires advanced configuration of sudo to allow restart & shutdown without password)
* Other
  - Internal fan controlled by GPIO pin - automatic work

# Source
https://github.com/astrojolo/astrolink4pi

# Requirements
* INDI available here http://indilib.org/download.html
* CMake >= 2.4.7

# Stellarmate installation prerequisites
```
sudo apt update
sudo apt install git
sudo apt-get install build-essential
sudo apt-get install cmake
sudo apt-get install libindi-dev
sudo apt-get install gpiod libgpiod-dev libgpiod-doc
sudo systemctl enable pigpiod
```

# Astroberry installation prerequisites
```
sudo apt update
sudo apt-get install cmake libindi-dev libgpiod-dev
sudo systemctl enable pigpiod
```

# AstroLink 4 Pi driver installation
```
git clone https://github.com/astrojolo/astrolink4pi
cd astrolink4pi
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
```
Or update to latest version:
```
cd ~/astrolink4pi/build/
git pull
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
```
You can install the drivers by running:
```
sudo make install
```
OR manually installing files by running:
```
sudo cp indi_astrolink4pi /usr/bin/
sudo cp indi_astrolink4pi.xml /usr/share/indi/
```
After these steps AstroLink 4 Pi driver will be visible in the Aux devices lists under **Astrojolo** group.

**Real Time clock enabling - version 2 and above**

To enable automatic synchronization of the RTC embedded in the versions 2 and above of AstroLink 4 Pi you need to edit the file
```
sudo nano /etc/rc.local
```
and add following line before exit 0 statement at the file end
```
echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-1/new_device
```
After restart the astroberry system time will be synchronized with embedded DS1307 clock.
Check hwclock help to find more options, like time adjustments and synchronization:
```
hwclock -h
```

# How to use it?
**version 1** Enable 1-Wire interface using raspi-config or adding 'dtoverlay=w1-gpio' to /boot/configure.txt for temperature compensation support (reboot required). 

**version 2 and 3** Enable I2C interface using raspi-config for sensor support (reboot required)

Run Kstars and select AstroLink 4 Pi (Aux section) in Ekos profile editor. Then start INDI server in Ekos with your profile, containg AstroLink 4 Pi drivers. Alternatively you can start INDI server manually by running:
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

# AstroLink 4 Pi images

![Photo](/images/al4pi-interior-v3.JPG)
