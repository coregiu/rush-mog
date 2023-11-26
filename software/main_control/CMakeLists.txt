cmake_minimum_required (VERSION 2.8.12)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set (ENV{MCPU} cortex-m3)

# set the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message("Debug build.")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
  message("Release build.")
endif()

# set search path for CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake/Modules)

# include OpenOCD module, but make it not required
find_package(Openocd)

# Project specific settings
set(application_name "ago_maincontrol")

cmake_policy(SET CMP0048 NEW)
project (${application_name} VERSION 1.2.3 LANGUAGES C CXX ASM)

configure_file(${CMAKE_SOURCE_DIR}/src/inc/version.h.in
  ${CMAKE_SOURCE_DIR}/src/inc/version.h)

# platform-specific configurations
include(CMake/platform.cmake)

# STM32-specific settings
set(DEVICE_FAMILY STM32F10X_MD)

# Speed of external resonator
set(HSE_VALUE 8000000)

# Used linker file (just for testing)
get_filename_component(LINKER_SCRIPT linker/stm32f103c8.ld ABSOLUTE)

# Use LTO? WARNING: This might break the code
#set(ADDITIONAL_CORE_FLAGS -flto)

# OpenOCD config file
get_filename_component(OPENOCD_CONFIG tools/stm32f103c8-jlink.cfg ABSOLUTE)

# add libraries
add_subdirectory(STM32F1)
add_subdirectory(ThirdParty/FreeRTOS)

# add main project
add_subdirectory(src)
