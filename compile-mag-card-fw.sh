#!/bin/bash
west build -b mcxn947_breakoutcard/mcxn947/cpu0 -p -- -DBOARD_ROOT=../../common
exit $?
