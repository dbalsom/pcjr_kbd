# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/pico_sdk_import.cmake)

project(pico_jr_kbd C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2 CACHE STRING "Pico board target")
endif()

pico_sdk_init()

add_executable(pico_jr_kbd
    main.c
    usb_descriptors.c
)

target_include_directories(pico_jr_kbd PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(pico_jr_kbd PRIVATE
    pico_stdlib
    hardware_gpio
    tinyusb_device
    tinyusb_board
)

pico_enable_stdio_usb(pico_jr_kbd 0)
pico_enable_stdio_uart(pico_jr_kbd 0)
pico_add_extra_outputs(pico_jr_kbd)
