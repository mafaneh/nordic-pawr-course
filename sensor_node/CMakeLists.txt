# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(endpoint_eink_pawr)

target_sources(app PRIVATE
    src/main.c
    src/app/ble_interface.c
    src/app/epaper_display.c
    src/app/sensor_interface.c
    src/app/sensor_node_sm.c)
