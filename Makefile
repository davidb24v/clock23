BOARD_TAG    = uno
ARDUINO_PORT = /dev/ttyUSB1
#ARDUINO_PORT = /dev/ttyACM0
ARDUINO_LIBS = ShiftPWM Wire RTClib
MONITOR_BAUDRATE = 115200

include ~/Arduino-Makefile/Arduino.mk

