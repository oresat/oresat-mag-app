# oresat-mag-app

OreSat Zephyr app for the mag card.

This card hosts both the magnetorquers and the magnetometers.

A separate reaction-wheel board with its own app will be
physically attached to the mag card.

## How To Build

For the Zephyr 4.2.0 application underway in mag card Zephyr branch 2026 Q2
build invocation which depends on an NXP MCU "pack" is:

```shell
west flash -r pyocd --tool-opt=--pack=./scripts/NXP.MCXN947_DFP.19.0.0.pack
```

## Zephyr Shell and I2C

```
uart:~$ i2c scan flexcomm0_lpi2c0
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:             -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
1 devices found on flexcomm0_lpi2c0
uart:~$ i2c read flexcomm0_lpi2c0 0x68 0x75
00000000: 47 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00 |G....... ........|
```
