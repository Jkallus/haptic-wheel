idf_component_register(
                    SRCS main.cpp
                    INCLUDE_DIRS "."
                    PRIV_INCLUDE_DIRS
                    	../components/arduino/tools/sdk/esp32/include/freertos/include/freertos
                    	../components/arduino/tools/sdk/esp32s2/include/arduino_tinyusb/include
                    	../components/arduino/tools/sdk/esp32s2/include/arduino_tinyusb/tinyusb/src
                    	../components/arduino/tools/sdk/esp32/include/freertos/include/freertos
                    	../components/arduino/tools/sdk/esp32s2/include/arduino_tinyusb/tinyusb/src)
