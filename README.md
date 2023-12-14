# AstroLink 4 Pi
AstroLink 4 Pi device is the astroimaging setup controller based on the Raspberry Pi module. It contains focusing motor controller, switchable power outputs, regulated dew-cap heaters outputs, regulated voltage output and power monitoring function. Selection of sensors can be connected to this device - temperature, humidity, dew point, sky temperature / cloud coverage and sky brightness.
### AstroLink 4 Pi works both with Raspberry Pi 4 and 5. 

> [!NOTE]
> The most recent driver version and Raspberry Pi 5 support is only for device revision 4 and later (starting from October 2023). For earlier revisions see the section below the AstroLink 4 Pi features.

> [!NOTE]
> Raspberry Pi 5 is based on new OS Bookworm. Make sure the software you use is available for this new OS before you upgrade to RPi5.

## Device
https://shop.astrojolo.com/astrolink-4-computers/


# AstroLink 4 Pi driver installation
## Requirements
* INDI http://indilib.org/download.html
* lgpio https://abyz.me.uk/lg/download.html 

### Required packages
```
sudo apt update
sudo apt install git build-essential cmake libindi-dev
```
### INDI driver installation
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

After these steps AstroLink 4 Pi driver will be visible in the Aux devices lists under **Astrojolo** group.

### Real-time clock
To enable automatic synchronization of the RTC embedded in versions 2 and above of AstroLink 4 Pi you need to edit the file
```
sudo nano /etc/rc.local
```
and add the following line before the exit 0 statement at the file end
```
echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-1/new_device
```
After restarting the astroberry system time will be synchronized with embedded DS1307 clock.
Check hwclock help to find more options, like time adjustments and synchronization:
```
hwclock -h
```

# AstroLink 4 Pi features:
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
  - Automatic temperature compensation based on temperature sensor
  - Humidity / dew point / sky temperature / cloud coverage / sky brighntess sensors support (version 3 and later)
  - Stepper movement abort
  - 6-pin RJ12 stepper output
  - embedded real-time clock (version 2 and later)
  - voltage, current, and energy monitor (version 4 and later)
* Power outputs
  - Two switchable 12V DC outputs, 5A max each
  - One permanent 12V DC output
  - Two PWM-regulated RCA outputs, 3A max each
  - One adjustable DC output 3-10V, 1.5A max
  - Configurable labels
* Other
  - Internal fan controlled by GPIO pin - automatic work

## Source
https://github.com/astrojolo/astrolink4pi

![Photo](/images/astrolink4pi-banner.jpg)

# Devices revision 3 and earlier
INDI driver AstroLink 4 Pi revision 3 and earlier must be installed from the tag https://github.com/astrojolo/astrolink4pi/releases/tag/3.0.0

> [!NOTE]
> Revision 3 and earlier of AstroLink 4 Pi works only with Raspberry Pi 4

Additional packages required:
```
sudo apt install gpiod libgpiod-dev libgpiod-doc
sudo systemctl enable pigpiod
```
In AstroLink 4 Pi revision 3 and earlier internal fan is not controlled by the INDI driver. You need to open Raspberry configuration and switch on the fan on GPIO 13 (Performance tab).

# How to use it?
**version 1 and later** - Enable 1-Wire interface using raspi-config or adding 'dtoverlay=w1-gpio' to /boot/configure.txt for temperature compensation support (reboot required). 

**version 2 and later** - Enable I2C interface using raspi-config for sensor support (reboot required)

Run Kstars and select AstroLink 4 Pi (Aux section) in the Ekos profile editor. Then start the INDI server in Ekos with your profile, containing AstroLink 4 Pi drivers. Alternatively, you can start INDI server manually by running:
```
indi_server indi_astrolink4pi
```
Start KStars with Ekos, connect to your INDI server and enjoy!

Note that your user account needs proper access rights to /dev/gpiochip0 device. By default, you can read/write only if you run the driver as root or a user who is a member of gpio group. Add your user to gpio group by running ```sudo usermod -a -G gpio $USER```

To use restart/shutdown functionality add this line to your /etc/sudoers file or /etc/sudoers.d/010_astroberry-nopasswd (this assumes you run INDI server as an astroberry user):
```
astroberry ALL=(ALL) NOPASSWD: /sbin/reboot, /sbin/poweroff
```

For custom labels, you need to save the configuration and restart the driver after changing the relays' labels.

![Photo](/images/al4pi-interior-v3.JPG)
