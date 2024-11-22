# Enable sysbuild
cmake_minimum_required(VERSION 3.21)
find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(power_meter_cleat)

# Ensure HCI_RPMsg is built as a child image for Bluetooth communication
if (CONFIG_BT_RPMSG)
    set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# Application's sources
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE inc)
