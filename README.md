STM32F10X library for MMDVM_HS, ADF7021 hotspot project. Use this library when you need to build with Makefile. This package is not necessary if you are using stm32duino.

This is the STM32F10X_Lib directory structure with just the needed code for MMDVM_HS.
Copy this STM32F10X_Lib directory inside the MMDVM_HS source code project. You have to
conserve the STM32F10X_Lib folder name: MMDVM_HS/STM32F10X_Lib

In order to run the MMDVM_HS firmware correctly, there are the following modifications
already done in this package:

1) STM32F10X_Lib/Device/startup/startup_stm32f10x.c :
- Added C++ global constructors support
- Changed initial stack pointer

2) STM32F10X_Lib/Device/stm32f10x_conf.h :
- Peripheral header file inclusions

3) STM32F10X_Lib/usb
- Maple Serial-USB support (Virtual COM), only for STM32F103 devices. Most of this code comes from Roger Clark's STM32Duino project. There are modifications in order to use the latest ST libraries. Also, this USB library includes Roger's fix to run this project with USB in Windows.

4) STM32F10X_Lib/utils
- The binary bootloader and utilities in this folder are from Roger Clark's STM32duino project.
