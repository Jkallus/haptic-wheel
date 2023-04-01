# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS components/Arduino-FOC)

set(CONFIG_ARDUINO_VARIANT "esp32s3")


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(haptic-knob)
