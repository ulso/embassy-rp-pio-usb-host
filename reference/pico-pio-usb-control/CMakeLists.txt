cmake_minimum_required(VERSION 3.13)

if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Configure with -DPICO_SDK_PATH=/path/to/pico-sdk")
endif()
if(NOT PICO_PIO_USB_DIR)
    message(FATAL_ERROR "Configure with -DPICO_PIO_USB_DIR=/path/to/Pico-PIO-USB")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_pio_usb_control C CXX ASM)

pico_sdk_init()
add_subdirectory(${PICO_PIO_USB_DIR} pico_pio_usb)

add_executable(pico_pio_usb_control main.c)
target_link_libraries(pico_pio_usb_control PRIVATE pico_stdlib pico_pio_usb)
pico_add_extra_outputs(pico_pio_usb_control)
