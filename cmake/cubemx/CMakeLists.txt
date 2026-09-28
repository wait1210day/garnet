# Project-owned adapter. Never include/add_subdirectory the generated CMake:
# it injects sources into CMAKE_PROJECT_NAME instead of providing a library.
get_filename_component(cubemx_root
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/cubemx" ABSOLUTE)

set(cubemx_definitions
    USE_FULL_LL_DRIVER
    USE_HAL_DRIVER
    STM32G474xx
    $<$<CONFIG:Debug>:DEBUG>
)
set(cubemx_include_dirs
    Inc
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
)
set(cubemx_board_sources
    Src/main.c
    Src/gpio.c
    Src/app_freertos.c
    Src/adc.c
    Src/comp.c
    Src/dac.c
    Src/fdcan.c
    Src/hrtim.c
    Src/i2c.c
    Src/usart.c
    Src/dma.c
    Src/stm32g4xx_it.c
    Src/stm32g4xx_hal_msp.c
    Src/stm32g4xx_hal_timebase_tim.c
    Src/sysmem.c
    Src/syscalls.c
)
set(cubemx_driver_sources
    Src/system_stm32g4xx.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_pwr.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_comp.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac_ex.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_hrtim.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_i2c.c
    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usart.c
)
set(cubemx_freertos_sources
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# Keep parsing and diagnostics in a standalone, standard-library-only script.
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
set(cubemx_checker "${CMAKE_CURRENT_LIST_DIR}/../../tools/check_cubemx_manifest.py")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${cubemx_root}/cmake/stm32cubemx/CMakeLists.txt"
    "${cubemx_checker}")
execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${cubemx_checker}"
    RESULT_VARIABLE cubemx_check_result
    OUTPUT_VARIABLE cubemx_check_output
    ERROR_VARIABLE cubemx_check_error
)
if(NOT "${cubemx_check_result}" STREQUAL "0")
    message(FATAL_ERROR "${cubemx_check_output}${cubemx_check_error}")
endif()

foreach(manifest IN ITEMS cubemx_include_dirs cubemx_board_sources
        cubemx_driver_sources cubemx_freertos_sources)
    list(TRANSFORM ${manifest} PREPEND "${cubemx_root}/")
endforeach()

add_library(cubemx_headers INTERFACE)
target_include_directories(cubemx_headers INTERFACE ${cubemx_include_dirs})
target_compile_definitions(cubemx_headers INTERFACE ${cubemx_definitions})
target_link_libraries(cubemx_headers INTERFACE garnet_mcu)

add_library(cubemx_board OBJECT ${cubemx_board_sources})
add_library(cubemx_drivers OBJECT ${cubemx_driver_sources})
add_library(cubemx_freertos OBJECT ${cubemx_freertos_sources})
foreach(component IN ITEMS cubemx_board cubemx_drivers cubemx_freertos)
    target_link_libraries(${component} PUBLIC cubemx_headers)
endforeach()

# Forward objects explicitly: transitive OBJECT-library usage requirements
# alone do not place their object files on the final executable's link line.
add_library(cubemx INTERFACE)
add_library(cubemx::cubemx ALIAS cubemx)
target_link_libraries(cubemx INTERFACE
    cubemx_board cubemx_drivers cubemx_freertos
    $<TARGET_OBJECTS:cubemx_board>
    $<TARGET_OBJECTS:cubemx_drivers>
    $<TARGET_OBJECTS:cubemx_freertos>
)
