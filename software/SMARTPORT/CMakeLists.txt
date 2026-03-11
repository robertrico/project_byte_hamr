cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "../../../../pico-sdk-master")
set(PICO_TOOLCHAIN_PATH "/Applications/ARM/bin")
set(PICO_BOARD pico_w)

include(pico_sdk_import.cmake)

project(picoport C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 0)

pico_sdk_init()

# --- SD Card + FatFS (carlk3 library) ---
add_subdirectory(lib/sd_fat_spi/FatFs_SPI build_fat)

# --- Common sources ---
set(COMMON_SRCS fm_decode.S)
set(COMMON_LIBS pico_stdlib hardware_pio pico_multicore)

# --- SmartPort device sources ---
set(SP_SRCS
    sp_cmd.c
    sp_asm.S
    sp_encode.c
    sp_decode_pkt.c
    sp_data.c
    sd_block.c
    hw_config.c
)

# === Main firmware ===
set(TARGET picoport)

add_executable(${TARGET} main.c ${COMMON_SRCS} ${SP_SRCS})
pico_generate_pio_header(${TARGET} ${CMAKE_CURRENT_LIST_DIR}/fm_tx.pio)
pico_generate_pio_header(${TARGET} ${CMAKE_CURRENT_LIST_DIR}/fm_rx.pio)
target_link_libraries(${TARGET} ${COMMON_LIBS} FatFs_SPI)
pico_add_extra_outputs(${TARGET})
pico_enable_stdio_usb(${TARGET} 1)
pico_enable_stdio_uart(${TARGET} 1)

# === Test firmware ===
set(TEST_TARGET picoport_test)

add_executable(${TEST_TARGET} test_main.c ${COMMON_SRCS})
target_compile_definitions(${TEST_TARGET} PRIVATE TEST_MODE=1)
pico_generate_pio_header(${TEST_TARGET} ${CMAKE_CURRENT_LIST_DIR}/fm_tx.pio)
pico_generate_pio_header(${TEST_TARGET} ${CMAKE_CURRENT_LIST_DIR}/fm_rx.pio)
target_link_libraries(${TEST_TARGET} ${COMMON_LIBS})
pico_add_extra_outputs(${TEST_TARGET})
pico_enable_stdio_usb(${TEST_TARGET} 1)
pico_enable_stdio_uart(${TEST_TARGET} 1)
