![Platform](https://img.shields.io/badge/Raspberry%20Pi-4%20%7C%205-green)
![INDI](https://img.shields.io/badge/INDI-2.1.3+-blue)
![License](https://img.shields.io/badge/license-GPL--3.0-orange)

# 🔭 AstroLink 4 Pi

## 📚 Table of Contents

- [Quick Start](#-quick-start)
- [Compatibility Matrix](#-compatibility-matrix)
- [Features](#-features)
- [Installation](#-installation)
  - [Prerequisites](#-prerequisites)
  - [Build & Install](#-build--install)
  - [Enable Interfaces](#-enable-interfaces)
  - [RTC Setup](#-rtc-setup-if-applicable)
- [INDI / Ekos Setup](#-indi--ekos-setup)
- [Permissions](#-permissions)
- [Troubleshooting](#-troubleshooting)
- [Summary](#-summary)

AstroLink 4 Pi is an INDI driver for AstroLink hardware, designed to simplify and integrate astrophotography setups.  
It provides control over power distribution, focuser operation, environmental monitoring, and device telemetry — all in a single compact system.

🛒 [Product page](https://shop.astrojolo.com/product/astrolink-4-pi/) <br>
🔗 [INDI Library](https://indilib.org/)

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
([see Prerequisites section below for full installation path](#-prerequisites))
```bash
git clone https://github.com/astrojolo/astrolink4pi.git
cd astrolink4pi
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

🔗 [For StellarMate OS 2.0 specific steps check this section](#-stellarmate-os-20-setup)<br>
🔗 [For AstroArch specific steps check this section](#-astroarch-setup)

After installation, the driver will appear in:
```
INDI → Auxiliary devices → AstroLink 4 Pi
```
> [!IMPORTANT]
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

## ⚡ Features

### 🔭 Focuser
- Stepper motor control
- Absolute and relative positioning
- DRV8825 driver support for Moonlite / Robofocus / AstroLink geared unipolar steppers and bipolar microstepping up to 1/32
- Forward / Reverse direction configuration
- Customizable maximum absolute position (steps)
- Customizable maximum focuser travel (mm)
- Backlash compensation
- Speed control
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

---

## ⭐ Summary

AstroLink 4 Pi combines:
- 🔌 Power management  
- 🔭 Focuser control  
- 🌡️ Environmental sensing  
- ⚙️ Full INDI integration  

➡️ All in one compact astrophotography controller

---

## 🧪 StellarMate OS 2.0 Setup

---

## 🧪 AstroArch Setup

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
