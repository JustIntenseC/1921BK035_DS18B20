#-- Service --------------------------------------------------------------------
SET(CMAKE_TOOLCHAIN_FILE ../../../../tools/gcc_cm4f.cmake)
ENABLE_LANGUAGE(ASM)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8.0)
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt." )
ENDIF()
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#-- Project config -------------------------------------------------------------
PROJECT(blinky)                     # Project name
SET(MCUNAME         K1921VK035)     # MCU name
SET(BOARDNAME       NIIET-BB-K1921VK035)  # Board name
SET(OSECLK_VAL      16000000)       # OSECLK value in Hz (0 if disconnected)
SET(SYSCLK          PLL)            # SYSCLK source: PLL, OSI, OSE
SET(CKO             NONE)           # Clockout source: PLL, OSI, OSE, NONE (no clockout)
SET(RETARGET        1)              # Enable(1) or disable(0) printf retarget
SET(RETARGET_USE    UART)           # ITM or UART printf retarget
SET(PRINTF_FLOAT    1)              # Enable(1) or disable(0) printf float support
SET(SCANF_FLOAT     1)              # Enable(1) or disable(0) scanf float support

#-- Defines --------------------------------------------------------------------
ADD_DEFINITIONS(-D${MCUNAME})
ADD_DEFINITIONS(-DOSECLK_VAL=${OSECLK_VAL})
ADD_DEFINITIONS(-DSYSCLK_${SYSCLK})
ADD_DEFINITIONS(-DCKO_${CKO})

#-- Project paths --------------------------------------------------------------
SET(APP_PATH                ../app)
SET(PLATFORM_PATH           ../../../../platform)
SET(DEVICE_SRC_PATH         ${PLATFORM_PATH}/Device/NIIET/${MCUNAME}/Source)
SET(DEVICE_INC_PATH         ${PLATFORM_PATH}/Device/NIIET/${MCUNAME}/Include)
SET(CMSIS_CORE_INC_PATH     ${PLATFORM_PATH}/CMSIS/Core/Include)
SET(RETARGET_SRC_PATH       ${PLATFORM_PATH}/retarget/Template/${MCUNAME})
SET(RETARGET_INC_PATH       ${PLATFORM_PATH}/retarget/Template/${MCUNAME})
# SET(HARDWARE_PATH           ../../../../hardware)
# # SET(BSP_SRC_PATH            ${HARDWARE_PATH}/bsp/${BOARDNAME})
# # SET(BSP_INC_PATH            ${HARDWARE_PATH}/bsp/${BOARDNAME})

# -- openocd -------------------------------------------------------------
set(OPENOCD_EXE "C:/Users/a.faizullin/Desktop/openocd-win/bin/openocd.exe")
set(OPENOCD_SCRIPTS_DIR "C:/Users/a.faizullin/Desktop/openocd-win/scripts/")
set(OPENOCD_CONFIG "${OPENOCD_SCRIPTS_DIR}board/connect_stlink.cfg")

#-- Include dirs ---------------------------------------------------------------
INCLUDE_DIRECTORIES(${APP_PATH})
INCLUDE_DIRECTORIES(${CMSIS_CORE_INC_PATH})
INCLUDE_DIRECTORIES(${DEVICE_INC_PATH})
INCLUDE_DIRECTORIES(${RETARGET_INC_PATH})
INCLUDE_DIRECTORIES(${BSP_INC_PATH})

#-- Sources list ---------------------------------------------------------------
FILE(GLOB_RECURSE APP_SRC ${APP_PATH}/*.c ${RETARGET_SRC_PATH}/*.c ${DEVICE_SRC_PATH}/*.c)
LIST(APPEND APP_SRC ${DEVICE_SRC_PATH}/GCC/startup_${MCUNAME}.S)
LIST(APPEND APP_SRC ${PLATFORM_PATH}/retarget/retarget.c)

#-- Options --------------------------------------------------------------------
IF(RETARGET STREQUAL "1")
    ADD_DEFINITIONS(-DRETARGET)
    ADD_DEFINITIONS(-DRETARGET_USE_${RETARGET_USE})
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-builtin")
ENDIF()
IF(PRINTF_FLOAT STREQUAL "1")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u_printf_float")
ENDIF()
IF(SCANF_FLOAT STREQUAL "1")
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u_scanf_float")
ENDIF()

#-- Linker script --------------------------------------------------------------
SET(LDSCRIPT ${CMAKE_SOURCE_DIR}/${DEVICE_SRC_PATH}/GCC/${MCUNAME}.ld)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LDSCRIPT} -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--print-memory-usage")

#-- Project linking ------------------------------------------------------------
ADD_EXECUTABLE(${PROJECT_NAME}.elf ${APP_SRC})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}.elf)

#-- Custom commands ------------------------------------------------------------
# ADD_CUSTOM_COMMAND(TARGET ${PROJECT_NAME}.elf POST_BUILD
#         COMMAND ${CMAKE_OBJCOPY} "-Oihex" ${PROJECT_NAME}.elf ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
#         COMMAND ${CMAKE_OBJCOPY} "-Obinary" ${PROJECT_NAME}.elf ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
#         COMMAND ${CMAKE_OBJDUMP} "-DS" ${PROJECT_NAME}.elf > ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.dasm
#         COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf)

ADD_CUSTOM_COMMAND(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} "-Oihex" $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} "-Obinary" $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJDUMP} "-DS" $<TARGET_FILE:${PROJECT_NAME}.elf> > ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.dasm
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
)


# Создаём цель для прошивки
add_custom_target(
    flash
    COMMAND ${OPENOCD_EXE}
        -s ${OPENOCD_SCRIPTS_DIR}
        -f ${OPENOCD_CONFIG}
        -c "program C:/Users/a.faizullin/Desktop/PrimeriVoronezh/K1921BK035/niietcm4-k1921vkx_sdk-45893dd9ab5d/projects/NIIET-BB-K1921VK035/blinky/GCC/build/blinky.hex verify reset exit"
    DEPENDS blinky.hex
    COMMENT "Flashing ${PROJECT_NAME}.hex using OpenOCD"
)

