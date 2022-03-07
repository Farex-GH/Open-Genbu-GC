cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(genbu-fw)

pico_sdk_init()

add_executable(genbu-fw
    main.c
    proc_queue.c
    board_io.c
    descriptors.c
    usb_gamepad.c
    usb_driver.c
    ws2812.c
)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(genbu-fw ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/pio)

pico_enable_stdio_usb(genbu-fw 0)
pico_enable_stdio_uart(genbu-fw 1)

pico_add_extra_outputs(genbu-fw)

target_link_libraries(genbu-fw
    pico_stdlib
    hardware_pio
    pico_multicore)

target_include_directories(genbu-fw PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# add url via pico_set_program_url
#example_auto_set_url(genbu-fw)
