# Oresat MCXN Breakout

This repository contains the board files for the Oresat MCXN947 protocard (breakout board).

It currently enables flexcomm4 UART for the console, I2C0 with a Bosch Sensortec BME280,
DAC0, and ADC0 channels 1A and 2A.

TODO: move this into the common repo, make it generic for the breakout board,
then define app-specific device tree changes in a device tree overlay in any apps that need to
use this board.

Follow the [README installation instructions](https://github.com/plskeggs/oresat-zephyr-common) for installation.
