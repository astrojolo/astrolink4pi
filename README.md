# AstroLink 4 Pi
AstroLink 4 Pi device is the astroimaging setup controller based on the Raspberry Pi module. It contains focusing motor controller, switchable power outputs, regulated dew-cap heaters outputs, regulated voltage output and power monitoring function. Selection of sensors can be connected to this device - temperature, humidity, dew point, sky temperature / cloud coverage and sky brightness.
### AstroLink 4 Pi works both with Raspberry Pi 4 and 5. 

> [!NOTE]
> This INDI driver works with revisions 3 and newer of AstroLink 4 Pi device (the ones with RJ sensor socket). For earlier revisions see the section below the AstroLink 4 Pi features.

> [!NOTE]
> Raspberry Pi 5 is based on new OS Bookworm. Make sure the software you use is available for this new OS before you upgrade to RPi5. Currently, Stellarmate OS 1.8.0 supports RPi5 and AstroArch was tested successfully.

## Device
https://shop.astrojolo.com/astrolink-4-computers/


# AstroLink 4 Pi driver installation
## Requirements
* INDI http://indilib.org/download.html
* lgpio https://abyz.me.uk/lg/download.html 
* I<sup>2</sup>C and SPI support must be enabled in Raspberry configuration

For AstroArch system there are few additional steps required that are listed below the AstroLink 4 features section.

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
You can install the drivers by running:
```
sudo make install
```
After these steps AstroLink 4 Pi driver will be visible in the Aux devices lists under **Astrojolo** group.

### Real-time clock
To enable automatic synchronization of the RTC embedded in AstroLink 4 Pi you need to edit the file
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
  - Internal fan controlled by GPIO pin - controlled automatically by INDI driver

## Source
https://github.com/astrojolo/astrolink4pi

![Photo](/images/astrolink4pi-banner.jpg)

# Devices revision 2 and earlier
INDI driver AstroLink 4 Pi revision 2 and earlier must be installed from the tag https://github.com/astrojolo/astrolink4pi/tree/3.0.0

> [!NOTE]
> Revision 2 and earlier of AstroLink 4 Pi works only with Raspberry Pi 4

Additional packages required:
```
sudo apt install gpiod libgpiod-dev libgpiod-doc
sudo systemctl enable pigpiod
```
AstroLink 4 Pi driver installation (for revisions 2 and older)
```
git clone https://github.com/astrojolo/astrolink4pi
cd astrolink4pi
mkdir build && cd build
git checkout tags/3.0.0
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
sudo make install
```
In AstroLink 4 Pi revision 2 and earlier internal fan is not controlled by the INDI driver. You need to open Raspberry configuration and switch on the fan on GPIO 13 (Performance tab).

### AstroArch only specific tasks
Update system and install packages:
```
update-astroarch
pacman -S unzip cmake python python3 python-setuptools python3-setuptools swig
```

RTC clock enabling is described at https://github.com/devDucks/astroarch . RTC device name is _ds1307_. 

Before compiling _lgpio_ find the following line in _Makefile_ file:
```
prefix ?= /usr/local
```
and update to
```
prefix ?= /usr
```

Create additional group and add user _astronaut_ to them:
```
sudo groupadd gpio
sudo usermod -a -G gpio astronaut
sudo usermod -a -G i2c astronaut
```
Create _/etc/udev/rules.d/99-gpio.rules_ file with content:
```
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP:="gpio", MODE:="0660"
```


# Revisions matrix
### Revision 4
* Works with the most recent INDI driver version from _main_.
* Works with Raspberry Pi 4 or 5.
* Requires _lgpio_ library for GPIO control.
* Requires I<sup>2</sup>C enabled.
### Revision 3
* Works with the most recent INDI driver version from _main_.
* Works with Raspberry Pi 4 or 5.
* Requires _lgpio_ library for GPIO control.
* Requires I<sup>2</sup>C and SPI enabled.
### Revision 2
* Works with tag 3.0 of INDI driver.
* Works with Rasbperry Pi 4 only.
* Requires _pigpio_ library for GPIO control.
* Requires I<sup>2</sup>C and SPI enabled.
### Revision 1
* Works with tag 3.0 of INDI driver.
* Works with Rasbperry Pi 4 only.
* Requires _pigpio_ library for GPIO control.
* Requires 1-Wire enabled.

# How to use it?
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
