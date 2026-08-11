# oresat-mag-app

OreSat Zephyr app for the mag card.

This card hosts both the magnetorquers and the magnetometers.

A separate reaction-wheel board with its own app will be
physically attached to the mag card.

# Building and flashing
Ensure you are in the `mag` directory (`cd src/oresat/firmware/apps/mag`) prior to building.

> NOTE:
>   The `mcxn947_mag_card` is the default. It normally does not need to be specified as shown below,
>   except for the important exception of enabling MCUboot. In that case, it is mandatory for Zephyr 4.2.0.

| Board         | Build Example                                         |
| ------------- | ----------------------------------------------------- |
| mcxn947_mag_card | `west build -p always -b mcxn947_mag_card/mcxn947/cpu0 -- -DCONFIG_MCUBOOT_ALLOWED=n` |
| mcxn947_mag_card with shell | `west build -p always -b mcxn947_mag_card/mcxn947/cpu0 -- -DEXTRA_CONF_FILE=overlay_shell.conf -DCONFIG_MCUBOOT_ALLOWED=n` |
| mcxn947_mag_card with MCUboot | `west build -p always -b mcxn947_mag_card/mcxn947/cpu0 --sysbuild -- -DBOARD_ROOT=$PWD` |

> NOTE: the section below only gives general instructions. Specific steps below (like for setting the CAN node id) are self-contained
>   in the section.

# Building and flashing without the bootloader
This runs the build and the flash faster when working in the lab.
However, this will of course not be able to be remotely updated over
CAN.

## First build

For the first build, or when previously running with the bootloader:
```
$ west build -p
$ west flash --erase
```

> NOTE: the `--erase` option will lose the settings data that stores a
>   non-default CAN node id.

## Subsequent builds:
```
$ west build
$ west flash
```

#### Flashing

Flashing is done via [probe-rs](probe.rs). Follow their [installation instructions](https://probe.rs/docs/getting-started/installation/).
Once installed, follow their [probe setup instructions](https://probe.rs/docs/getting-started/probe-setup/).

If you have previously installed probe-rs and are on an old version (v<0.32), make sure you follow their uninstalling procedure and use their most up-to-date release.

Flash the build using `west flash`.

To fully erase before flashing, do `west flash --erase`


# Setting the CAN node id
This can be done through a terminal with the Zephyr shell enabled. This **must be set** to the
correct number expected by the C3 card. Use `oresat-configs cards` to check which number to use.

This requires three steps in practice:
1. Build and flash with the `overlay_shell.conf` configuration file applied
1. Enter the CAN node id in the terminal
1. Build and flash again without the `overlay_shell.conf` without erasing the chip

## Enabling the shell
Do a build with the `overlay_shell.conf` file as a parameter to `west build`:
```
$ west build -- -DEXTRA_CONF_FILE=overlay_shell.conf
```
Flash as explained earlier.

## Using the shell
The terminal will continue to show whatever log messages are currently enabled.
But it will also show, at the bottom of the scroll, a prompt:
```
uart:~$
```

> NOTE: with the shell enabled, the normal control loop taking setpoints over CAN for
>   magnetorquer current is disabled, and replaced with test code. See the commands starting with **mt**.

Commands:
- `help` for help. There are many commands included by Zephyr itself not listed below.

- `nodeid`<enter> to see the current nodeid.
  Change the node id by entering `nodeid <N>` where N is the desired node id in decimal.
  The value is then stored in the settings partition, so as long as you do not do a full chip erase,
  it will be remembered.

- `gyrocal [reset]`<enter> to do initial gyroscope calibration without the reset option.
  The unit must be stationary.
  If the gyroscope has already been calibrated, add the optional reset parameter: `gyrocal reset`<enter>.
  Once calibrated, the values obtained are considered an offset from 0 to be subtracted from future
  measurements. Like the `nodeid`, the calibration values are stored in the settings partition.

- `mtmode [<num>|name] [<axis>]`<enter> to get or set the test mode.
  - 0 TM_OFF: only other shell commands active; useful to stop the TM_LOOP_RAMP mode
  - 1 TM_ADC: read and display ADC sense channels
  - 2 TM_CURRENT: read and display current sense measurements
  - 3 TM_LOOP: run mt control loop
  - 4 TM_LOOP_RAMP: run mt control loop while ramping target

- `mtpwm [<axis>] [<new duty cycle value>]`<enter> to get or set an axis's PWM output value. This is only
  for experimentation in the lab, and is not stored to settings.

- `frqpwm [<axis>] [<new frequency in Hz>]`<enter> to get or set an axis's PWM frequency. This is only for
  experimentation in the lab, and is not stored to settings.

- `ua2pwm <axis> <target current in uA>`<enter> to set the output current for an axis.
  This exercises the closed loop control of PWM to set a current. This is similar to the non-shell normal
  operating mode, except the target current is set via the terminal, rather than over CAN.

## Rebuilding to remove shell
Build again, using whatever options you need as explained in the document.

> NOTE: do not use the `--erase` parameter with `west flash` or you will lose the change you
> just made with the shell.

# Building and flashing with bootloader
```
$ west build -p -b mcxn947_mag_card/mcxn947/cpu0 --sysbuild -- -DBOARD_ROOT=$PWD
$ west flash --erase
```
This will build two binaries: one for the bootloader, and one for the application.
The flashing step will be done automatically in two parts to flash these two binaries.

> NOTE: The app flashing step will run slowly: this is normal.

## CAN flashing script
The `flash_canopen.py` script can be used to test flashing over CAN on the desktop.

```
    $python3 flash_canopen.py --help
usage: flash_canopen.py [-h] [--serial-port SERIAL_PORT] [--channel CHANNEL] [--bitrate BITRATE] [--node-id NODE_ID] [--bin BIN_PATH] [--block-transfer] [--download-buffer-size DOWNLOAD_BUFFER_SIZE]
                        [--status-timeout STATUS_TIMEOUT] [--bootup-timeout BOOTUP_TIMEOUT] [--sdo-timeout SDO_TIMEOUT] [--sdo-retries SDO_RETRIES] [--confirm] [--no-confirm] [--request-crc]
                        [--throttle-delay THROTTLE_DELAY] [--debug]

options:
  -h, --help            show this help message and exit
  --serial-port SERIAL_PORT
  --channel CHANNEL
  --bitrate BITRATE
  --node-id NODE_ID
  --bin BIN_PATH
  --block-transfer
  --download-buffer-size DOWNLOAD_BUFFER_SIZE
  --status-timeout STATUS_TIMEOUT
  --bootup-timeout BOOTUP_TIMEOUT
  --sdo-timeout SDO_TIMEOUT
  --sdo-retries SDO_RETRIES
  --confirm
  --no-confirm
  --request-crc
  --throttle-delay THROTTLE_DELAY
  --debug               Enable verbose CAN logging
```

This example flashes the mag app firmware to a CANable adapter on ttyACM1, where the mag card is running
the default CAN id of 124.
```
$ python3 flash_canopen.py --serial-port /dev/ttyACM1 --bin  build/mag/zephyr/zephyr.signed.bin --node-id 124
```

> NOTE: The CAN id must be changed to the proper one for this card before the
>  C3 can access it.
