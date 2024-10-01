#!/bin/bash

echo rm -r build
rm -r build

echo mkdir build
mkdir build

echo cmake -D FREERTOS_KERNEL_PATH=/path/to/rp2040-freertos-lora/FreeRTOS-Kernel -D PICO_SDK_PATH=/path/to/pico_projects/pico-sdk -B ./build
cmake -D FREERTOS_KERNEL_PATH=/path/to/rp2040-freertos-lora/FreeRTOS-Kernel -D PICO_SDK_PATH=/path/to/pico_projects/pico-sdk -B ./build

echo make -C build
make -C build
