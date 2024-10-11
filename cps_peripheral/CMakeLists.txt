# Enable sysbuild
cmake_minimum_required(VERSION 3.21)
find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(your_project_name)

# Ensure HCI_RPMsg is built as a child image for Bluetooth communication
if (CONFIG_BT_RPMSG)
    set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# Add your application's sources
target_sources(app PRIVATE src/main.c)
