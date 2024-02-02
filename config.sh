#!/bin/bash

# Toolchain:
export GNU_INSTALL_ROOT=/usr/local/gcc-arm-none-eabi-9-2020-q2-update/bin/
export GNU_VERSION=9.3.1
export GNU_PREFIX=arm-none-eabi

# nRF5 SDK:
export SDK_ROOT=$(pwd)/sdk/nRF5_SDK_17.1.0_ddde560

# Application:
# export BUILD_PROJECT=kSense
export BUILD_PROJECT=kSenseQi
# export DEBUG=1

## Deinit application globals
# unset BUILD_PROJECT
# unset DEBUG

echo $GNU_INSTALL_ROOT
echo $GNU_VERSION
echo $GNU_PREFIX
echo $SDK_ROOT
echo $BUILD_PROJECT
echo $DEBUG

# vscode:
code ws.code-workspace