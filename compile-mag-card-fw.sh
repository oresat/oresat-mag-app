#!/bin/bash
# west build -b mcxn947_breakoutcard/mcxn947/cpu0 -p -- -DBOARD_ROOT=../../common
west build -p always -b mcxn947_mag_card/mcxn947/cpu0 -- -DCONFIG_MCUBOOT_ALLOWED=n
exit $?
