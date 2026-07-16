# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(RPI-OS C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# library configuration (override if necessary, see pico-st7735 for a complete list)
set(SPI_TFT_PORT "spi0" CACHE STRING "TFT spi port-name")
set(SPI_TFT_CS   "3"    CACHE STRING "TFT CS  pin number")
set(SPI_TFT_DC   "12"   CACHE STRING "TFT DC  pin number")
set(SPI_TFT_RST  "2"   CACHE STRING "TFT RST pin number")
set(TFT_OPTIONS TFT_ENABLE_GREEN TFT_ENABLE_RESET TFT_ENABLE_TEXT TFT_ENABLE_SHAPES
                TFT_ENABLE_ROTATE TFT_ENABLE_SCROLL
            CACHE STRING "TFT options/functions")

#program configuration
set(SPI_PORT "spi0" CACHE STRING "spi port-name")
set(SPI_RX   "16"   CACHE STRING "RX  pin number")
set(SPI_TX   "19"   CACHE STRING "TX  pin number")
set(SPI_SCK  "18"   CACHE STRING "SCK pin number")



add_compile_definitions(SPI_PORT=${SPI_PORT})
add_compile_definitions(SPI_RX=${SPI_RX})
add_compile_definitions(SPI_TX=${SPI_TX})
add_compile_definitions(SPI_SCK=${SPI_SCK})
foreach(opt IN LISTS TFT_OPTIONS)
    message(">>> using: ${opt}")
    add_compile_definitions(${opt})
endforeach(opt)

# add preprocessor-constant DEBUG for Debug-builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_VERBOSE_MAKEFILE 1)
  add_compile_definitions(DEBUG)
else()
endif()


# Add executable. Default name is the project name, version 0.1

add_executable(RPI-OS RPI-OS.c minmea.c)

pico_set_program_name(RPI-OS "RPI-OS")
pico_set_program_version(RPI-OS "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(RPI-OS 0)
pico_enable_stdio_usb(RPI-OS 1)

add_subdirectory(extern/pico-st7735)

target_compile_definitions(RPI-OS PRIVATE
    MINMEA_INCLUDE_COMPAT           # ← needed for non-Unix (Pico)
    timegm=mktime 
)

# Add the standard library to the build
target_link_libraries(RPI-OS
        pico_stdlib
        hardware_spi
        st7735
        hardware_rtc 
)


# Add the standard include files to the build
target_include_directories(RPI-OS PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(RPI-OS)

