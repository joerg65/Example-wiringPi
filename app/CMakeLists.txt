# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

cmake_minimum_required(VERSION 3.4.0)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds it for you.
# Gradle automatically packages shared libraries with your APK.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

set(distribution_DIR /home/joerg/Development/Ndk)
set(wiring_DIR wiringPi)

include_directories( ${distribution_DIR}/${wiring_DIR}/wiringPi
        ${distribution_DIR}/${wiring_DIR}/devLib
        src/main/cpp)


######################################################################
# library wiringPi
######################################################################

add_library( wiringPi SHARED

        # Provides a relative path to your source file(s).
        # Associated headers in the same location as their source
        # file are automatically included.

        ${distribution_DIR}/${wiring_DIR}/wiringPi/ads1115.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp23008.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp4802.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/piHiPri.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/sr595.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/bmp180.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp23016.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/odroidc1.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/odroidn2.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/piThread.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/wiringPi.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp23017.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/odroidc2.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/odroidc4.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/odroidhc4.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/drcSerial.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp23s08.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/odroidn1.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/wiringPiI2C.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/ds18b20.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp23s17.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/sn3218.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/wiringPiSPI.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/htu21d.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp3002.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/odroidxu3.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/softPwm.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/wiringSerial.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/max31855.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp3004.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/pcf8574.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/softServo.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/wiringShift.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/max5322.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/mcp3422.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/pcf8591.c
        ${distribution_DIR}/${wiring_DIR}/wiringPi/softTone.c)

target_link_libraries( wiringPi ${log-lib} )

######################################################################
# end of library wiringPi
######################################################################


######################################################################
# library wpi_android
######################################################################

#add_subdirectory(src/main/cpp)
add_library (wpi_android SHARED
        src/main/cpp/wpi_android.c
        src/main/cpp/si1132.c
        src/main/cpp/bme280.c
        src/main/cpp/bme280-i2c.c )

target_link_libraries( wpi_android wiringPi ${log-lib} )

######################################################################
# end of library wpi_android
######################################################################


######################################################################
# library wiringPiDev
######################################################################

add_library (wiringPiDev SHARED
              ${distribution_DIR}/${wiring_DIR}/devLib/ds1302.c
              ${distribution_DIR}/${wiring_DIR}/devLib/maxdetect.c
              ${distribution_DIR}/${wiring_DIR}/devLib/piNes.c
              ${distribution_DIR}/${wiring_DIR}/devLib/gertboard.c
              ${distribution_DIR}/${wiring_DIR}/devLib/piFace.c
              ${distribution_DIR}/${wiring_DIR}/devLib/lcd128x64.c
              ${distribution_DIR}/${wiring_DIR}/devLib/lcd.c
              ${distribution_DIR}/${wiring_DIR}/devLib/scrollPhat.c
              ${distribution_DIR}/${wiring_DIR}/devLib/piGlow.c )

target_link_libraries( wiringPiDev wiringPi ${log-lib} )

######################################################################
# end of library wiringPiDev
######################################################################

######################################################################
# executable gpio
######################################################################

set(EXECUTABLE_OUTPUT_PATH      "${CMAKE_CURRENT_SOURCE_DIR}/src/main/assets/${ANDROID_ABI}")

add_executable (gpio
              ${distribution_DIR}/${wiring_DIR}/gpio/gpio.c
              ${distribution_DIR}/${wiring_DIR}/gpio/readall.c)

target_link_libraries( gpio wiringPi wiringPiDev ${log-lib} )

######################################################################
# end of executable gpio
######################################################################