# Toolchain file for the Arm GNU toolchain (arm-none-eabi-gcc).
# Looks on PATH first, then in ~/tools/arm-gnu-toolchain-*/bin.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

file(GLOB _arm_hints "$ENV{HOME}/tools/arm-gnu-toolchain-*/bin")
find_program(ARM_GCC arm-none-eabi-gcc HINTS ${_arm_hints} REQUIRED)
get_filename_component(_arm_bin "${ARM_GCC}" DIRECTORY)

set(CMAKE_C_COMPILER   "${_arm_bin}/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${_arm_bin}/arm-none-eabi-gcc")
set(CMAKE_OBJCOPY      "${_arm_bin}/arm-none-eabi-objcopy")
set(CMAKE_SIZE         "${_arm_bin}/arm-none-eabi-size")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
