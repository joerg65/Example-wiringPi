# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

cmake_minimum_required(VERSION 3.4.1)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds it for you.
# Gradle automatically packages shared libraries with your APK.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/wiringPi
                     ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi )


######################################################################
# library wiringPi
######################################################################

add_library( wiringPi SHARED

             # Provides a relative path to your source file(s).
             # Associated headers in the same location as their source
             # file are automatically included.
             src/main/cpp/wiringPi/wiringPi/wiringSerial.c
             src/main/cpp/wiringPi/wiringPi/wiringPi.c
             src/main/cpp/wiringPi/wiringPi/wiringShift.c
             src/main/cpp/wiringPi/wiringPi/piHiPri.c
             src/main/cpp/wiringPi/wiringPi/piThread.c
             src/main/cpp/wiringPi/wiringPi/wiringPiSPI.c
             src/main/cpp/wiringPi/wiringPi/wiringPiI2C.c
             src/main/cpp/wiringPi/wiringPi/softPwm.c
             src/main/cpp/wiringPi/wiringPi/softTone.c
             src/main/cpp/wiringPi/wiringPi/mcp23008.c
             src/main/cpp/wiringPi/wiringPi/mcp23016.c
             src/main/cpp/wiringPi/wiringPi/mcp23017.c
             src/main/cpp/wiringPi/wiringPi/mcp23s08.c
             src/main/cpp/wiringPi/wiringPi/mcp23s17.c
             src/main/cpp/wiringPi/wiringPi/sr595.c
             src/main/cpp/wiringPi/wiringPi/pcf8574.c
             src/main/cpp/wiringPi/wiringPi/pcf8591.c
             src/main/cpp/wiringPi/wiringPi/mcp3002.c
             src/main/cpp/wiringPi/wiringPi/mcp3004.c
             src/main/cpp/wiringPi/wiringPi/mcp4802.c
             src/main/cpp/wiringPi/wiringPi/mcp3422.c
             src/main/cpp/wiringPi/wiringPi/max31855.c
             src/main/cpp/wiringPi/wiringPi/max5322.c
             src/main/cpp/wiringPi/wiringPi/sn3218.c
             src/main/cpp/wiringPi/wiringPi/drcSerial.c )

target_link_libraries( wiringPi ${log-lib} )

######################################################################
# end of library wiringPi
######################################################################


######################################################################
# library wpi_android
######################################################################

add_library (wpi_android SHARED

              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/wpi_android.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/si1132.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/bme280.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/bme280-i2c.c )

target_link_libraries( wpi_android wiringPi ${log-lib} )

######################################################################
# end of library wpi_android
######################################################################


######################################################################
# library wiringPiDev
######################################################################

add_library (wiringPiDev SHARED

              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/devLib/ds1302.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/devLib/maxdetect.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/devLib/piNes.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/devLib/gertboard.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/devLib/piFace.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/devLib/lcd128x64.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/devLib/lcd.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/devLib/piGlow.c )

target_link_libraries( wiringPiDev wiringPi ${log-lib} )

######################################################################
# end of library wiringPiDev
######################################################################

######################################################################
# executable gpio
######################################################################

set(EXECUTABLE_OUTPUT_PATH      "${CMAKE_CURRENT_SOURCE_DIR}/src/main/assets/${ANDROID_ABI}")

add_executable (gpio

              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/gpio/gpio.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/gpio/extensions.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/gpio/readall.c
              ${CMAKE_CURRENT_SOURCE_DIR}/src/main/cpp/wiringPi/gpio/pins.c )

target_link_libraries( gpio wiringPi ${log-lib} )

######################################################################
# end of executable gpio
######################################################################