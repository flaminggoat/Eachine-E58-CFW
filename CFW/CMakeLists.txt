SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake-arm-embedded/toolchain-arm-none-eabi.cmake")

PROJECT(e58)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
ENABLE_LANGUAGE(ASM)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/pan159.ld")
set(OBJECT_GEN_FLAGS "-mcpu=cortex-m0 -mfloat-abi=soft")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OBJECT_GEN_FLAGS}" CACHE INTERNAL "C Compiler options")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OBJECT_GEN_FLAGS}" CACHE INTERNAL "C++ Compiler options")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${OBJECT_GEN_FLAGS}" CACHE INTERNAL "ASM Compiler options")
# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mfloat-abi=soft -T${LINKER_SCRIPT}" CACHE INTERNAL "Linker options")

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Device/Nuvoton/Mini58Series/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/StdDriver/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/StdDriver/driver
)

SET(PROJECT_SOURCES
    main.c
    startup.c
    Library/Device/Nuvoton/Mini58Series/Source/system_Mini58Series.c
    Library/StdDriver/src/gpio.c
    Library/StdDriver/src/clk.c
    Library/StdDriver/src/i2c.c
    Library/StdDriver/src/pwm.c
)

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES})

# add_custom_command(TARGET e58
#         POST_BUILD
#         COMMAND /usr/bin/arm-none-eabi-objcopy -O binary ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.bin
#         )