rem Location of GCC Cross-compiler: https://developer.arm.com/downloads/-/gnu-r
set GNU_GCC=C:/Program Files (x86)/GNU Arm Embedded Toolchain/9 2020-q2-update
set GNU_INSTALL_ROOT=%GNU_GCC%/bin
set GNU_VERSION="9.3.1"
set GNU_PREFIX="arm-none-eabi"

rem Location of Nordic SDK
set SDK_ROOT=%cd%/sdk/nRF5_SDK_17.1.0_ddde560

rem Select which project variant to build
rem set BUILD_PROJECT=kSense
set BUILD_PROJECT=kSenseQi

rem Location of Nordic Command Line tools (nrfjprog) : https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download
set NRF_TOOLS=C:/Program Files/Nordic Semiconductor/nrf-command-line-tools/bin

rem Activate this line to set the 'Debug' build config
rem set DEBUG=1

"%LOCALAPPDATA%/Programs/Microsoft VS Code/Code.exe" ws.code-workspace

