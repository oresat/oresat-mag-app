#
#  Project:  RM3100 Zephyr driver demo
#
#  SPDX-License-Identifiers: Apache-2.0, MIT license
#

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rm3100_demo)

target_sources(app PRIVATE
    src/main.c
)
