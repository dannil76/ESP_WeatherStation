PROGRAM = mqtt_client

EXTRA_COMPONENTS = extras/paho_mqtt_c
EXTRA_COMPONENTS += extras/i2c extras/bmp280
EXTRA_COMPONENTS += extras/onewire extras/ds18b20
EXTRA_COMPONENTS += extras/si7021
EXTRA_COMPONENTS += extras/rboot-ota

ESPPORT = /dev/tty.usbserial

include ../../esp-open-rtos/common.mk
