#!/bin/bash

# Note:  (1) Zephyr 4.2.99 flash runner calls openocd this way:
#
# /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/bin/openocd -f /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts/interface/stlink.cfg -f /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts/target/stm32f0x.cfg -s /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts  '-c init' '-c targets' -c 'reset init' -c 'flash write_image erase /home/ted/projects/zephyr-project/psas-ers-firmware/samples/hello-world/build/zephyr/zephyr.hex' -c 'reset run' -c shutdown

# Note:  (2) comment out following two functions, at least until resolving the
#  preference to expand a shell variable that's presently within single
#  quotes.  That syntax comes from above Zephyr runner example in note (1).

# function flash_only()
# {
# 	/opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/bin/openocd \
# 	-f /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts/interface/stlink.cfg \
# 	-f /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts/target/stm32f0x.cfg \
# 	-s /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts \
# 	'-c init' \
# 	'-c targets' \
# 	-c 'reset init' \
# 	-c 'flash write_image erase /home/ted/projects/zephyr-project/psas-ers-firmware/samples/hello-world/build/zephyr/zephyr.hex' \
# 	-c "flash write_image erase ${PWD}/build/zephyr/zephyr.hex" \
# 	-c 'reset run' \
# 	-c shutdown
# }

# function flash_and_enter_debugger()
# {
# 	/opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/bin/openocd \
# 	-f /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts/interface/stlink.cfg \
# 	-f /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts/target/stm32f0x.cfg \
# 	-s /opt/zephyr-sdk-0.16.5-1/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts \
# 	'-c init' \
# 	'-c targets' \
# 	-c 'reset init' \
# 	-c 'flash write_image erase /home/ted/projects/zephyr-project/psas-ers-firmware/samples/hello-world/build/zephyr/zephyr.hex' \
# 	-c 'reset run'
# }

function script_exit()
{
    exit $1
}

function goodbye_and_exit()
{
    echo "$SCRIPT_LONG_NAME done"
    echo
    script_exit
}

function flash_with_psas_recovery_board_options()
{
	# OPENOCD_CONFIG_PATH=/home/ted/projects/psas/psas-avionics/lv3.1-recovery/controlSystem/RecoveryBoard/firmware/toolchain
	# OPENOCD_CONFIG_FILE=oocd.cfg
        FIRMWARE_IMAGE=${PWD}/build/zephyr/zephyr.hex
	openocd \
        -f ${OPENOCD_CONFIG_FILE_PATH}/${OPENOCD_CONFIG_FILE} \
        -c "program ${FIRMWARE_IMAGE} verify reset exit"
}

function flash_with_psas_options_and_debug()
{
	# OPENOCD_CONFIG_PATH=/home/ted/projects/psas/psas-avionics/lv3.1-recovery/controlSystem/RecoveryBoard/firmware/toolchain
	# OPENOCD_CONFIG_FILE=oocd.cfg
        FIRMWARE_IMAGE=${PWD}/build/zephyr/zephyr.hex
	openocd \
        -f $1/${OPENOCD_CONFIG_FILE} \
        -c "program ${FIRMWARE_IMAGE} verify reset"
}

function usage()
{
    echo "Call flash-manually.sh with:"
    echo
    echo "-c, --config-file . . to specify pyocd configuration file to use"
    echo "-d, --debug . . . . . to flash and to start debugging server"
    echo "-f, --flash . . . . . to flash image to hardware"
    echo "-h, --help  . . . . . to show this help message"
    echo
}

#-----------------------------------------------------------------------
# - SECTION - starting point akin to int main
#-----------------------------------------------------------------------

OPENOCD_CONFIG_FILE_PATH=../../scripts
OPENOCD_CONFIG_FILE=oocd.cfg

PYOCD_REQUESTED_ACTION="f"   # 'flash' as opposed to 'debug'
SCRIPT_REQUESTED_ACTION="n"  # 'none' until user request parsed

echo "Zephyr openocd flash wrapper starting"
echo "current dir is $PWD"

OPTS=$(getopt -o c:dfhp: --long config-file:,config-path:,debug,flash,help \
	-n 'flash-manually.sh' -- "$@")

if [ $? -ne 0 ]; then
  echo "Failed to parse options" >&2
  usage
fi

## Reset the positional parameters to the parsed options
eval set -- "$OPTS"

## Process the options
while true; do
  case "$1" in
    -c | --config-file)
      OPENOCD_CONFIG_FILE="$2"
      shift 2
      ;;
    -p | --config-path)
      OPENOCD_CONFIG_FILE_PATH="$2"
      shift 2
      ;;
    -d | --debug)
      PYOCD_REQUESTED_ACTION="debug"
      shift
      ;;
    -f | --flash)
      PYOCD_REQUESTED_ACTION="flash"
      shift
      ;;
    -h | --help)
      echo "- DEBUG - got option to show 'help'"
      SCRIPT_REQUESTED_ACTION="show_help"
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "$0 Failed to parse options, exiting early!"
      exit 1
      ;;
  esac
done

if [ $SCRIPT_REQUESTED_ACTION = "show_help" ]; then
    usage
    goodbye_and_exit
fi

echo "- DEV 0330 BEGIN - "
echo
echo "Parsed following variables:"
echo "  - config file path:     '$OPENOCD_CONFIG_FILE_PATH'"
echo "  - config file name:     '$OPENOCD_CONFIG_FILE'"
echo "  - pyocd reque' action:  '$PYOCD_REQUESTED_ACTION'"
echo "  - script requ' action:  '$SCRIPT_REQUESTED_ACTION'"
echo
echo "- DEV 0330 END - "

if [ $PYOCD_REQUESTED_ACTION = "flash" ]; then
    echo "- DEV 0330 - Got request to run 'pyocd' to flash and run firmware"
    flash_with_psas_recovery_board_options
elif [ $PYOCD_REQUESTED_ACTION = "debug" ]; then
    echo "- DEV 0330 - Got request to run pyocd to debug firmware"
    flash_with_psas_options_and_debug $OPENOCD_CONFIG_FILE_PATH
else
    echo "- DEV 0330 - Failed to parse supported pyocd action."
    echo "    Should be one of 'debug' and 'flash'"
fi

# if [ "$1" == "w" ]; then
#     echo "Calling openocd with options to flash and then exit openocd . . ."
#     # flash_only
#     flash_with_psas_recovery_board_options
# elif [ "$1" == "d" ]; then
#     echo "Calling openocd with options to flash, start and maintain a gdb server . . ."
#     # flash_and_enter_debugger
#     flash_with_psas_options_and_debug
#
#

exit $?
