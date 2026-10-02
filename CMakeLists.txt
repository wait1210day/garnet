cmake_minimum_required(VERSION 3.22)

# A plain `cmake -S . -B ...` also selects the project's cross toolchain.
# An explicit toolchain (including one supplied by a preset) takes precedence.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake"
        CACHE FILEPATH "Cross-compilation toolchain")
endif()

project(garnet LANGUAGES C CXX ASM)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build configuration" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g3")

# Shared by application, generated board code, HAL/LL and FreeRTOS.
add_library(garnet_mcu INTERFACE)
target_compile_options(garnet_mcu INTERFACE
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    "$<$<COMPILE_LANGUAGE:C,CXX>:-Wall;-fdata-sections;-ffunction-sections;-fstack-usage>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti;-fno-exceptions;-fno-threadsafe-statics>"
    "$<$<COMPILE_LANGUAGE:ASM>:-x;assembler-with-cpp>"
)
target_link_options(garnet_mcu INTERFACE
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)

# our custom adapter for cubemx,
# never adding the generated CubeMX project directly
add_subdirectory(cmake/cubemx)

add_executable(garnet
    platform/startup_stm32g474xx.s

    src/base/utils.cpp
    src/base/panic.cpp

    src/drivers/ll_helper.cpp
    src/drivers/i2c_bus.cpp
    src/drivers/uart.cpp

    src/services/fumi.cpp
    src/services/hermes.cpp
    src/services/janus.cpp

    src/control/ctrloop.cpp
    src/control/adc_optimizer.cpp

    src/app_main.cpp
)

target_include_directories(garnet PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

set_target_properties(garnet PROPERTIES SUFFIX ".elf")
target_link_libraries(garnet PRIVATE cubemx::cubemx m)

# HAL/LL and FreeRTOS remain without LTO.
# TODO(masshiroio): make everything compile with LTO
foreach(target IN ITEMS garnet cubemx_board)
    target_compile_options(${target} PRIVATE -flto -fno-strict-aliasing)
endforeach()

set(garnet_linker_script "${CMAKE_CURRENT_SOURCE_DIR}/platform/STM32G474XX_FLASH.ld")
target_link_options(garnet PRIVATE
    -flto -fno-strict-aliasing
    "-T${garnet_linker_script}"
    --specs=nano.specs
    "LINKER:-Map=$<TARGET_FILE_DIR:garnet>/garnet.map"
    LINKER:--gc-sections
    LINKER:--print-memory-usage

    LINKER:-u,_printf_float
)
set_property(TARGET garnet APPEND PROPERTY LINK_DEPENDS "${garnet_linker_script}")
set_property(TARGET garnet APPEND PROPERTY ADDITIONAL_CLEAN_FILES
    "$<TARGET_FILE_DIR:garnet>/garnet.map")

# Keep the CubeMX compatibility workaround for toolchains that infer libob.
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

option(GARNET_ENABLE_FLASH "Provide the OpenOCD flash target" ON)
if(GARNET_ENABLE_FLASH)
    find_program(OPENOCD_BIN openocd REQUIRED)
    add_custom_target(flash
        COMMAND "${OPENOCD_BIN}"
            -f interface/stlink.cfg
            -f target/stm32g4x.cfg
            -c "program {$<TARGET_FILE:garnet>} verify reset exit"
        DEPENDS garnet
        USES_TERMINAL
        VERBATIM
        COMMENT "Flashing garnet via OpenOCD"
    )
endif()
