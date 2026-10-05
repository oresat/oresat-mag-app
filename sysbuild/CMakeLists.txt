list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(BOARD "mcxn947_mag_card@1.1.0/mcxn947/cpu0")
find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sysbuild LANGUAGES)
