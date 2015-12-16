## https://github.com/sudar/Arduino-Makefile/blob/master/examples/MakefileExample/Makefile-example.mk

PROJECT_DIR       = .
CXXFLAGS_STD      = -std=c++11
USER_LIB_PATH    :=  $(PROJECT_DIR)/lib
CFLAGS_STD        = -std=gnu11

BOARD_TAG    = nano
MCU = atmega328p

AVRDUDE_ARD_BAUDRATE = 57600
AVRDUDE_ISP_BAUDRATE = 57600
MONITOR_PORT = /dev/tty.usbserial-AL011AWG

# https://github.com/sudar/Arduino-Makefile
include /usr/local/opt/arduino-mk/Arduino.mk


.PHONY: test
test:
	ARDUINO_DIR=$(ARDUINO_DIR) ruby test/test.rb


