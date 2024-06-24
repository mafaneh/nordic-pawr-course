# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

SET(SHIELD "waveshare_epaper_gdey0213b74")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_node)

FILE(GLOB app_sources src/*.c src/app/*.c)
target_sources(app PRIVATE ${app_sources})