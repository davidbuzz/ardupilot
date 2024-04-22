cmake_minimum_required(VERSION 3.9)

#message(STATUS "BUILDDIR FROM ENV: ${BUILDDIR}")
#message(STATUS "IDF_PATH FROM ENV: ${IDF_PATH}")

# trying to do: https://github.com/espressif/esp-idf/blob/master/examples/build_system/cmake/idf_as_lib/CMakeLists.txt#

#include($ENV{IDF_PATH}/tools/cmake/project.cmake)


# tip: 'CACHE INTERNAL' on set(xxx) commands makes the var available to subdirectories and every point after its defined
# # ie it becomes a 'make-shift global variable' 
# what this basically does is *COPY/SAVE* the passed-in TEMPORARY ENV vars that are only valid the first time cmake is run ( and are not in cache yet ) into a more permanent var 
if (NOT "$ENV{CCACHE_ENABLE}" STREQUAL "")
    set(CCACHE_ENABLE "1" CACHE INTERNAL "Copied from environment CCACHE_ENABLE")
    message(STATUS "CCACHE_ENABLE: ${CCACHE_ENABLE}")
endif()
if (NOT "$ENV{IDF_CCACHE_ENABLE}" STREQUAL "")
    set(CCACHE_ENABLE "1" CACHE INTERNAL "Copied from environment IDF_CCACHE_ENABLE to CCACHE_ENABLE ")
    message(STATUS "IDF_CCACHE_ENABLE->CCACHE_ENABLE: ${CCACHE_ENABLE}")
endif()
if (NOT "$ENV{BUILDDIR}" STREQUAL "")
    set(BUILDDIR "$ENV{BUILDDIR}" CACHE INTERNAL "Copied from environment BUILDDIR")
    message(STATUS "BUILDDIR: ${BUILDDIR}")
endif()
if (NOT "$ENV{BOARD_NAME}" STREQUAL "")
    set(BOARD_NAME "$ENV{BOARD_NAME}" CACHE INTERNAL "Copied from environment BOARD_NAME")
    message(STATUS "BOARD_NAME: ${BOARD_NAME}")
endif()
if (NOT "$ENV{VEHICLETYPE}" STREQUAL "")
    set(VEHICLETYPE "$ENV{VEHICLETYPE}" CACHE INTERNAL "Copied from environment VEHICLETYPE")
    message(STATUS "VEHICLETYPE: ${VEHICLETYPE}")
endif()


project(arduplane)
enable_language(C CXX ASM)

set(targets "esp32"  "esp32s3" )

if("${TARGET}" IN_LIST targets)
    # Include for ESP-IDF build system functions
    include($ENV{IDF_PATH}/tools/cmake/idf.cmake)
    # Create idf::{target} and idf::freertos static libraries
    idf_build_process("${TARGET}"
                    # try and trim the build; additional components
                    # will be included as needed based on dependency tree
                    #
                    # although esptool_py does not generate static library,
                    # processing the component is needed for flashing related
                    # targets and file generation
                    # WARNING leaving (eg) fatfs out here causes libraries/AP_Filesystem/AP_Filesystem_ESP32.h , which includes #include "esp_vfs_fat.h" to not be able to find the right symbols
                    COMPONENTS  
                        "${TARGET}"  #esp32s3
                        freertos
                        #esp_rom             # for asprintf
                        newlib               # for time.h?
                        console          
                        tcpip_adapter
                        fatfs               # for esp_vfs_fat.h etc
                        esp_adc_cal
                        nvs_flash
                        esptool_py
                        app_update
                        esp_timer     # for esp_timer.h:242:26: error: 'FILE' was not declared in this scope
                        lwip  #libraries/AP_HAL_ESP32/i2c_sw.c:34:../../modules/esp_idf/components/lwip/port/esp32/include/lwipopts.h: In function 'timeout_from_offered':../../modules/esp_idf/components/lwip/port/esp32/include/lwipopts.h:276:32: error: 'CONFIG_LWIP_DHCP_COARSE_TIMER_SECS' undeclared (first use in this function); did you mean 'DHCP_COARSE_TIMER_SECS'?
                        
                    SDKCONFIG ${CMAKE_CURRENT_LIST_DIR}/sdkconfig
                    BUILD_DIR ${CMAKE_BINARY_DIR})
else()
    message(WARNING "Unknown target ${TARGET}")
    # Create stubs for esp32 and freertos, stub::esp32 and stub::freertos
    #add_subdirectory(stubs/esp32)
    #add_subdirectory(stubs/freertos)
    #add_subdirectory(stubs/spi_flash)
    # throw error and stop
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(elf_file ${CMAKE_PROJECT_NAME}.elf)
add_executable(${elf_file} main/main.cpp)

# buzz  - the CmakeLists.txt in the libraries folder uses a add_library() call, and we just add_subdirectory() to include it
add_subdirectory(libraries) 
#include(libraries/CMakeLists.txt) - cant do this and the above, its one or the other

target_include_directories(${elf_file} PUBLIC
   ${CMAKE_CURRENT_SOURCE_DIR}/libraries
 )

 
# these two vars are equivalent.... its just CURRENT_EXE_NAME is used in the Vehicle CMakeLists.txt files.
set(CURRENT_EXE_NAME ${elf_file})


#having build the libraries, we can now include the vehicle specific code
include(${VEHICLETYPE}/CMakeLists.txt)  #eg VEHICLETYPE=ArduPlane

#.. once its all done we link it with the IDF below..

# Link the static libraries to the executable
if("${TARGET}" IN_LIST targets)
target_link_libraries(${elf_file} idf::freertos idf::spi_flash ARDULIBS)
#target_link_libraries(${elf_file} idf::freertos idf::spi_flash)
# Attach additional targets to the executable file for flashing,
    # linker script generation, partition_table generation, etc.
    idf_build_executable(${elf_file})
else()
    #target_link_libraries(${elf_file} stub::esp32 stub::freertos stub::spi_flash)
    message(WARNING "Unknown target ${TARGET}")
endif()

# [100%] Linking CXX static library libTHING.a
# [100%] Built target THING



# linker needs this. COMPILE_OPTIONS is bot both C and C++
#idf_build_set_property(COMPILE_OPTIONS "-mlongcalls" APPEND)
# this is c++ only:
# idf_build_set_property(CXX_COMPILE_OPTIONS "-Wdeprecated-copy" APPEND) 
# this is c only: 
# idf_build_set_property(C_COMPILE_OPTIONS "-Wdeprecated-copy" APPEND)


# export the executable target through a variable to CMakeLists.txt files in subdirectories.
# update the dependent sources.
#add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main/main.cpp )
# add_subdirectory(libraries)

# if (LINKER_SCRIPT)
# 	# if linker script is defined, make the target to depends on it,
# 	# so the target will be rebuilt when linker script was changed.
#     set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})
# endif()

# CXXFLAGS
#target_compile_options( ${PROJECT_NAME} PUBLIC -std=gnu++11 -fno-rtti -fno-exceptions -fno-threadsafe-statics -fno-use-cxa-atexit -fno-strict-aliasing   )

# set the compiler: /home/buzz/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch5-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-g++

# this is a direct cmake-ism, and isn't IDF specific, but can be used, and sees to be used at least for ardupiliot code.?
#target_compile_options(${PROJECT_NAME} PUBLIC -Wno-reorder)


add_definitions(-I./${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../../../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir

add_definitions(-I../ArduPlane) # so main.cpp can find ../ArduPlane/Plane.h as "Plane.h"

add_definitions(-I./config)
add_definitions(-I../config)
add_definitions(-I../../config)
add_definitions(-I../../../config)
add_definitions(-I../../../../config)
add_definitions(-I../../../../../config)

add_definitions(-I./)
add_definitions(-I../)
add_definitions(-I../../)
add_definitions(-I../../../)
add_definitions(-I../../../../)
add_definitions(-I../../../../../)

# add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_ESP32)
# add_definitions(-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_ESP32_S3EMPTY) 
# add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_${VEHICLETYPE})
# #add_definitions(-DALLOW_DOUBLE_MATH_FUNCTIONS) - defined only in specific files its needed in - warning: "ALLOW_DOUBLE_MATH_FUNCTIONS" redefined  #define ALLOW_DOUBLE_MATH_FUNCTIONS <command-line>: note: this is the location of the previous definition

# # as close to waf as possible:
# add_definitions(-fcheck-new)
# add_definitions(-fsingle-precision-constant)
# add_definitions(-Wno-psabi)
# add_definitions(-std=gnu++11) #warning: command line option '-std=gnu++11' is valid for C++/ObjC++ but not for C

# add_definitions(-fdata-sections)
# add_definitions(-ffunction-sections)
# add_definitions(-fno-exceptions)
# add_definitions(-fsigned-char)
# add_definitions(-nostdlib)

# add_definitions(-Wno-reorder) # error: 'AP_HAL::HAL::serial_array' will be initialized after [-Werror=reorder]

# add_definitions(-DARDUPILOT_BUILD)
# add_definitions(-D__AP_LINE__=__LINE__)
# add_definitions(-DCYGWIN_BUILD)
# add_definitions(-DAP_CUSTOMCONTROL_ENABLED=0)
# add_definitions(-DAP_DDS_ENABLED=0)
# add_definitions(-DAP_SCRIPTING_CHECKS=1)

# if (BOARD_NAME STREQUAL "sitl")
#     add_definitions(-DAP_SIM_ENABLED=1)
# endif()

# add_definitions(-DENABLE_HEAP=0)
# add_definitions(-DENABLE_ONVIF=0)
# add_definitions(-DHAL_DEBUG_BUILD=1)
# add_definitions(-DHAL_HAVE_HARDWARE_DOUBLE=1)
# add_definitions(-DLUA_32BITS=1)


# add_definitions(-D__GNU_VISIBLE) #https://www.esp32.com/viewtopic.php?t=5799

# #add_definitions(-std=c++11) # for _GNU_SOURCE / asprintf as part of stdio.h no-good

# add_definitions(-DAP_BUILD_TARGET_NAME=${VEHICLETYPE})

# add_definitions(-DAP_GPS_ENABLED=1)

add_definitions(-Wno-trigraphs)  #warning: trigraph ??- ignored, use -trigraphs to enable [-Wtrigraphs]

add_definitions(-I${VEHICLETYPE})
add_definitions(-I../../${VEHICLETYPE})


# add_definitions(-I.)
# add_definitions(-I..)
# add_definitions(-I../..)
# add_definitions(-I../../..)

# add_definitions(-I./${BUILDDIR}/${BOARD_NAME}/esp-idf_build/config/)
# add_definitions(-I./${BUILDDIR}/config)
# add_definitions(-I../${BUILDDIR}/config)
# add_definitions(-I../../${BUILDDIR}/config)
# add_definitions(-I./${BUILDDIR}/bootloader/config)
# add_definitions(-I./bootloader/config)
# add_definitions(-I../bootloader/config)
# add_definitions(-I../../bootloader/config)
# add_definitions(-I./config)
# add_definitions(-I../config)
# add_definitions(-I../../config)

# add_definitions(-I./modules/esp_idf/components/xtensa/include/xtensa)
# add_definitions(-I../modules/esp_idf/components/xtensa/include/xtensa)
# add_definitions(-I../../modules/esp_idf/components/xtensa/include/xtensa)
# add_definitions(-I../../../modules/esp_idf/components/xtensa/include/xtensa)


# add_definitions(-Ilibraries)
# add_definitions(-Ilibraries/GCS_MAVLink)
# add_definitions(-I${VEHICLETYPE})
# add_definitions(-I../../${VEHICLETYPE})
# add_definitions(-Iesp-idf_build/config)

# add_definitions(-D__XTENSA__=1)


# add_definitions(-I.)
# add_definitions(-I..)
# add_definitions(-I../..)
# add_definitions(-I../../..)

# add_definitions(-I../../libraries)
# add_definitions(-I../../libraries/AP_Common)
# add_definitions(-I../../libraries/AP_Common/missing)
# add_definitions(-I../../../libraries/AP_Common/missing)

# #not sure which of these
# add_definitions(-I../../libraries/AP_HAL_ESP32/boards)
# add_definitions(-I../libraries/AP_HAL_ESP32/boards)
# add_definitions(-Ilibraries/AP_HAL_ESP32/boards)
# add_definitions(-I../libraries/AP_HAL_ESP32/boards)


#target_link_libraries(${PROJECT_NAME} PUBLIC 
#)


# target_include_directories(${PROJECT_NAME} PUBLIC
#   ${CMAKE_CURRENT_SOURCE_DIR}
#   ${CMAKE_CURRENT_SOURCE_DIR}/libraries
#   ${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_Common
#   ${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_Common/missing
#   ${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_HAL
#   ${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_HAL/board
#   ${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_HAL_ESP32
#   ${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_HAL_ESP32/boards
#   #${CMAKE_CURRENT_SOURCE_DIR}/${BUILDDIR}/sitl/libraries/GCS_MAVLink  #generated before this
#   ${CMAKE_CURRENT_SOURCE_DIR}/${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink  #generated before this

#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/xtensa/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/xtensa/esp32s3/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/newlib/platform_include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/freertos/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/freertos/include/esp_additions/freertos
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/freertos/port/xtensa/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/freertos/include/esp_additions
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_hw_support/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_hw_support/include/soc
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_hw_support/include/soc/esp32s3
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_hw_support/port/esp32s3
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_hw_support/port/esp32s3/private_include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/heap/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/log/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/lwip/include/apps
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/lwip/include/apps/sntp
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/lwip/lwip/src/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/lwip/lwip/src/include/lwip
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/lwip/port/esp32/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/lwip/port/esp32/include/arch
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/soc/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/soc/esp32s3
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/soc/esp32s3/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/hal/esp32s3/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/hal/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/hal/platform_port/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_rom/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_rom/include/esp32s3
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_rom/esp32s3
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_common/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_system/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_system/port/soc
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_system/port/public_compat
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/vfs/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_wifi/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_event/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_netif/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_eth/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/tcpip_adapter/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_phy/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_phy/esp32s3/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_ipc/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/app_trace/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_timer/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/driver/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/driver/esp32s3/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_pm/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_ringbuf/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/efuse/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/efuse/esp32s3/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/fatfs/diskio
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/fatfs/vfs
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/fatfs/src
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/wear_levelling/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/spi_flash/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/sdmmc/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/esp_adc_cal/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/nvs_flash/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/mbedtls/port/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/mbedtls/mbedtls/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/mbedtls/esp_crt_bundle/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/mbedtls/mbedtls/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/app_update/include
#     #${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/bootloader_support/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/modules/esp_idf/components/bootloader/include
#   )

# export the name of executable target via a variable to CMakeLists.txt files in subdirectories.
#set(CURRENT_EXE_NAME ${PROJECT_NAME})
# load and run the CMake code from subdirectories for current target.


# include(libraries/CMakeLists.txt)
# include(${VEHICLETYPE}/CMakeLists.txt)  #eg VEHICLETYPE=ArduPlane

# bootloader bootloader_support efuse esp32s3 esp_common esp_hw_support esp_rom esp_system esptool_py freertos hal log main micro-ecc newlib partition_table soc spi_flash xtensa

# # Link the static libraries to the executable
# target_link_libraries(${PROJECT_NAME}
#                 idf::bootloader
#                 #idf::bootloader_support
#                 idf::efuse
#                 idf::esp32s3
#                 idf::esp_common
#                 idf::esp_hw_support
#                 idf::esp_rom
#                 idf::esp_system
#                 idf::esptool_py
#                 idf::freertos
#                 idf::hal
#                 idf::log
#                 idf::main
#                 #idf::micro-ecc
#                 idf::newlib
#                 idf::partition_table
#                 idf::soc
#                 #idf::fatfs
#                 #idf::esp_adc_cal
#                 #idf::nvs_flash
#                 idf::spi_flash
#                 idf::xtensa
#                 #idf::tcpip_adapter
#                 #idf::app_update
#                 )


#include(ArduPlane/CMakeLists.txt)
#include(ArduSub/CMakeLists.txt)
#include(Blimp/CMakeLists.txt)
#include(ArduCopter/CMakeLists.txt)
#include(AntennaTracker/CMakeLists.txt)
#include(Rover/CMakeLists.txt)


# # if compiler is GNU gcc/g++, then generate *.bin & *.hex files.
# if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
#     # generate the hex file from the built target.
#     set(HEX_FILE ${PROJECT_NAME}.hex)
#     add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#         COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
#         COMMENT "Building ${HEX_FILE}...")

#     # generate the bin file from the built target.
#     set(BIN_FILE ${PROJECT_NAME}.bin)
#     add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#         COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${BIN_FILE}
#         COMMENT "Building ${BIN_FILE}...")
# endif()