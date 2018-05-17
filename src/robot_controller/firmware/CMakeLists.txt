cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(robot_controller
  SRCS robot_controller.cpp ${ROS_LIB_DIR}/time.cpp PWM.cpp
  HDRS include.h PWM.h
  BOARD atmega328
  PORT /dev/ttyUSB0
)