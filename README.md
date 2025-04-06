# Drone beacon

## Description

This repository contain the source code and schematic of the drone beacon that is mandatory for all flying models over 800gr in France.
The beacon use a ESP32 to send the model id over wifi and a gps module to know it position.

All model must be registered on [AlphaTango](https://alphatango.aviation-civile.gouv.fr/login.jsp)

This work is based on the work done by Arnaud Mazin and the source of it can be found [here](https://www.club-modelisme-saclay.fr/ameliorations-balise-signalement-electronique-modeles-reduits-volants/)

## Build, flash and monitor

### Install ESP-IDF tools

Follow the [get-started](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html) of the ESP IDF tools documentation.

For linux users
```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

mkdir -p ~/esp
cd ~/esp
git clone -b v5.4 --recursive https://github.com/espressif/esp-idf.git

cd ~/esp/esp-idf
./install.sh esp32
```

After that when you want to use the ESP-IDF tools you only need to source the export.sh script.
```
. $HOME/esp/esp-idf/export.sh
```

### Init submodule

```
git submodule update --init --recursive
```

### Beacon

```
cd source/beacon
idf.py build
idf.py flash
```

### Receiver

```
cd source/beacon
idf.py build
idf.py flash
```

### Monitor the serial output

```
idf.py monitor
```
To exit the monitor use Ctrl+T Ctrl+X
