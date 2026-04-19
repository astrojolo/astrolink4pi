![Platform](https://img.shields.io/badge/Raspberry%20Pi-4%20%7C%205-green)
![INDI](https://img.shields.io/badge/INDI-2.1.3+-blue)
![License](https://img.shields.io/badge/license-GPL--3.0-orange)

# 🔭 AstroLink 4 Pi

AstroLink 4 Pi is an INDI driver for AstroLink hardware, designed to simplify and integrate astrophotography setups.  
It provides control over power distribution, focuser operation, environmental monitoring, and device telemetry — all in a single compact system.

🛒 [Product page](https://shop.astrojolo.com/product/astrolink-4-pi/) <br>
🔗 [INDI Library](https://indilib.org/)

---
## ⚡ Features

AstroLink 4 Pi combines:
- 🔌 Power management  
- 🔭 Focuser control  
- 🌡️ Environmental sensing  
- ⚙️ Full INDI integration  

➡️ All in one compact astrophotography controller

### 🔭 Focuser
- Stepper motor control
- Absolute and relative positioning
- DRV8825 driver support for Moonlite / Robofocus / AstroLink geared unipolar steppers and bipolar microstepping up to 1/32
- Forward / Reverse direction configuration
- Customizable maximum absolute position (steps)
- Customizable maximum focuser travel (mm)
- Backlash compensation
- Speed/current/hold torque control
- Focuser info including: critical focus zone in μm, step size in μm, steps per critical focus zone
- Automatic temperature compensation based on temperature sensor

### 🔌 Power Outputs
- Two switchable 12V DC outputs, 5A max each
- One permanent 12V DC output
- Two PWM-regulated RCA outputs, 3A max each
- One adjustable DC output 3-10V, 1.5A max
- Configurable labels

### 🌡️ Sensors & Monitoring
- I2C environmental sensors
- Humidity / dew point / sky temperature / cloud coverage / sky brightness sensors support *(depending on revision)*
- Voltage and current monitoring *(depending on revision)*

### 🧠 System Integration
- Fully integrated with INDI ecosystem
- Works with NINA, KStars, Ekos, and other INDI clients


---

## ⚡ Quick Start

**Supported hardware**
- AstroLink 4 Pi

**Supported platforms**
- Raspberry Pi 4  
- Raspberry Pi 5 *AstroLink 4 Pi versions 3 and later*

**Required software**
- INDI **2.1.3 or newer**
- wiringPi https://github.com/WiringPi/WiringPi/tree/master?tab=readme-ov-file#installing

**Basic installation** 
```bash
git clone https://github.com/astrojolo/astrolink4pi.git
cd astrolink4pi
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

🔗 [Steps described in Prerequisities section must be applied](#-prerequisites)<br>
🔗 [For StellarMate OS 2.0 specific steps check this section](#-stellarmate-os-20-setup)<br>
🔗 [INDI Server helper script](#-indi-server-helper-script)<br>
🔗 [For AstroArch specific steps check this section](#-astroarch-setup)

After installation, the driver will appear in:
```
INDI → Auxiliary devices → AstroLink 4 Pi
```
> **⚠️ Important**<br>
> Once connected to the driver, configure the settings to match your requirements - mostly focusing motor settings and also adjust polling interval, so sensors will be updated more often than default 60 seconds.


---

## 📊 Compatibility Matrix

| Device Revision | Raspberry Pi | Driver Branch | Notes |
|----------------|-------------|--------------|------|
| Rev 4 | Pi 4 / Pi 5 | `main` | Full support |
| Rev 3 | Pi 4 / Pi 5 | `main` | Full support |
| Rev 2 | Pi 4 only | `main` | Legacy hardware |
| Rev 1 | Pi 4 only | `main` | Limited support |


---

## 🔧 Installation

### 📦 Prerequisites

Install required packages:

```bash
sudo apt update
sudo apt install -y \
  git cmake build-essential \
  libindi-dev 
```
**Wiring Pi installation**

```bash
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
```

**Adding current user access to SPI and I2C devices**

Check the group the devices are in:
```bash
ls -al /dev/spi*
ls -al /dev/i2c*
```
The group can be uucp, spi, or another. Add current user to the listed groups:
```bash  
sudo usermod -aG uucp $USER
```
Then reboot.

---

### 🛠️ Build & Install

```bash
git clone https://github.com/astrojolo/astrolink4pi.git
cd astrolink4pi
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

---

### 🔌 Enable Interfaces

Make sure required interfaces are enabled:

```bash
sudo raspi-config
```

Enable:
- I2C  
- SPI *(if required by your revision)*  

---

### ⏱️ RTC Setup (if applicable)

If your device revision includes RTC:

```bash
sudo nano /etc/rc.local
```

Add at the end:

```
echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-1/new_device
```

Then reboot:

```bash
sudo reboot
```

---
## 🧰 INDI Server Helper Script

This script (located in `server` folder) simplifies running a local INDI server by loading driver names from a configurable `profile.conf` file. It supports background execution, live debugging mode with real-time logs, and basic process management (start/stop/status/restart). If the configuration file does not exist, it is automatically created with a default setup.

## 🚀 Usage

```bash
./start_indi.sh start       # start INDI server in background
./start_indi.sh startlog    # start in foreground with live logs
./start_indi.sh stop        # stop the server
./start_indi.sh status      # check if running
./start_indi.sh restart     # restart the server
```
Edit `profile.conf` to change the list of INDI drivers. In Ekos, use a **Remote** profile and connect to *localhost:7624*.

---

## 🧪 StellarMate OS 2.0 Setup
<details>
<summary>Click to expand</summary>

Pacman **core** and **extra** repositories must be enabled:
```bash
sudo nano /etc/pacman.conf
```
and uncomment sections **core** and **extra**. Then install required packages:
```bash
sudo pacman -Syu
sudo pacman -Syu git
sudo pacman -Syu cmake
sudo pacman -Syu base-devel
```
Create rules set for **gpiomem** devices:
```bash
sudo nano /etc/udev/rules.d/99-zzz-astrolink4pi.rules
```
and put the line into this file:
```bash
KERNEL=="gpiomem*", GROUP="uucp", MODE="0660"
```
then reboot.<br><br>
**👉 Now you can install AstroLink 4 Pi INDI driver as described in the [Prerequisities section](#-prerequisites)**

**⚠️ Important**<br>
Since StellarMate 2.0 introduced Flatpak software containers, Kstars+Ekos now runs in isolated environment and cannot access INDI drivers from local system.<br>
Possible solutions are:
- install Kstars from regular distribution
- use Flatpak Kstars and run your own indiserver

👉 [Check INDI Server helper script](#-indi-server-helper-script)
</details>
---

## 🧪 AstroArch Setup
<details>
<summary>Click to expand</summary>

If you are using **AstroArch Linux**, additional steps may be required.

Typical adjustments:
- manual enabling of interfaces
- package differences
- RTC configuration

👉 Follow AstroArch-specific documentation if needed


Update system and install packages:

```bash
update-astroarch
sudo pacman -S unzip cmake python python3 python-setuptools swig
```

Add the following line to /boot/config.txt:

```bash
dtparam=spi=on
```

and modify

```bash
dtoverlay=i2c-rtc,ds1307
```

Create additional groups and add user astronaut to them:

```bash
sudo groupadd gpio
sudo groupadd spi
sudo usermod -a -G gpio astronaut
sudo usermod -a -G i2c astronaut
sudo usermod -a -G spi astronaut
```

Create /etc/udev/rules.d/99-gpio.rules file with content:

```bash
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP:="gpio", MODE:="0660"
SUBSYSTEM=="spidev", KERNEL=="spidev*", GROUP:="spi", MODE:="0660"
```

Then you may go directly to AstroLink 4 Pi INDI driver installation.
</details>
---

## ⚠️ Compatibility Notes

- This driver uses **wiringPi**, works correctly for Raspberry Pi 5 and 4
- Older INDI versions may not work correctly

---


## 🤝 Contributing

Contributions, bug reports, and suggestions are welcome!

If you find an issue:
- open a GitHub issue
- include logs and hardware revision
- describe your setup (Pi version, OS, INDI version)

---

## 📄 License

This project is released under the terms specified in the repository.