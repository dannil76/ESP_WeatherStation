# ESP8266 Weather Station
ESP8266 WeatherStation Firmware for the ESP_WeatherStation_v2.0_PCB (it also works with ESP_WeatherStation_PCB).

The ESP8266 Weather Station sends MQTT messages with temperature, humidity and air pressure to a MQTT server.

## Requirements

- esp-open-sdk
- esp-open-rtos
- xtensa-lx106-elf

## Installation / dependencies

### esptool

You need the python-based utility tool - esptool, to communicate with the ROM bootloader in Espressif ESP8266 & ESP32 series chips.

For easy installation I recommend using pyenv to go virtual with your python environment

```
$pipenv shell
$pipenv install esptool
$pipenv install setuptools
```