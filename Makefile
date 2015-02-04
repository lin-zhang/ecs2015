BOARD_TAG    = uno
LOCAL_CPP_SRCS = src/main.cpp src/hw_functions.cpp src/lcsm.cpp
ARDUINO_LIBS = Wire Servo PID_v1 LiquidCrystal_I2C MsTimer2
#THIRD_PARTY_LIBS=${HOME}/sketchbook/libraries
THIRD_PARTY_LIBS=repos/libraries
CPPFLAGS+=      -I /usr/share/arduino/libraries/Servo                   \
                -I /usr/share/arduino/libraries/Wire                    \
                -I ${THIRD_PARTY_LIBS}/LiquidCrystal_I2C                \
                -I ${THIRD_PARTY_LIBS}/MsTimer2                         \
                -I ${THIRD_PARTY_LIBS}/PID_v1

include ./Arduino.mk
