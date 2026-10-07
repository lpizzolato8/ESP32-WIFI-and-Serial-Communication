cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
project(stm32_pose_receiver C ASM)

set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(HAL ${LIB}/stm32f4xx-hal-driver)

if(NOT EXISTS ${HAL}/Src)
  message(FATAL_ERROR "HAL sources missing. Run: git submodule update --init --depth 1")
endif()

add_executable(receiver.elf
  src/main.c
  src/pose_rx.c
  src/stm32f4xx_it.c
  ${LIB}/cmsis-device-f4/Source/Templates/system_stm32f4xx.c
  ${LIB}/cmsis-device-f4/Source/Templates/gcc/startup_stm32f405xx.s
  ${HAL}/Src/stm32f4xx_hal.c
  ${HAL}/Src/stm32f4xx_hal_cortex.c
  ${HAL}/Src/stm32f4xx_hal_dma.c
  ${HAL}/Src/stm32f4xx_hal_flash.c
  ${HAL}/Src/stm32f4xx_hal_gpio.c
  ${HAL}/Src/stm32f4xx_hal_pwr_ex.c
  ${HAL}/Src/stm32f4xx_hal_rcc.c
  ${HAL}/Src/stm32f4xx_hal_uart.c
)

target_include_directories(receiver.elf PRIVATE
  src
  ${CMAKE_CURRENT_SOURCE_DIR}/../main        # shared pose_frame.h
  ${HAL}/Inc
  ${LIB}/cmsis-device-f4/Include
  ${LIB}/cmsis-core/CMSIS/Core/Include
)

target_compile_definitions(receiver.elf PRIVATE STM32F405xx USE_HAL_DRIVER)

set(MCU -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(receiver.elf PRIVATE ${MCU}
  -Wall -Wextra -ffunction-sections -fdata-sections
  $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>)
target_link_options(receiver.elf PRIVATE ${MCU}
  -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F405RG.ld
  --specs=nano.specs
  -Wl,--gc-sections -Wl,--no-warn-rwx-segments -Wl,-Map=receiver.map -Wl,--print-memory-usage)
target_link_libraries(receiver.elf PRIVATE m)

add_custom_command(TARGET receiver.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary receiver.elf receiver.bin
  COMMAND ${CMAKE_SIZE} receiver.elf)

# Flash over USB with the STM32 ROM bootloader: hold BOOT while plugging in.
add_custom_target(flash
  COMMAND dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D receiver.bin
  DEPENDS receiver.elf
  USES_TERMINAL)
