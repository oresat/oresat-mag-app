#!/bin/bash
west flash -r pyocd --tool-opt=--pack=./scripts/NXP.MCXN947_DFP.19.0.0.pack
exit $?
