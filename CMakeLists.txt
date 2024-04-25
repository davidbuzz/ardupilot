cmake_minimum_required(VERSION 3.9)

# trying to do: https://github.com/espressif/esp-idf/blob/master/examples/build_system/cmake/idf_as_lib/CMakeLists.txt#

#dont do this: include($ENV{IDF_PATH}/tools/cmake/project.cmake)


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

# idf-specific setup - we're "remembering" here, these values into the cache.
if (NOT "$ENV{IDF_PATH}" STREQUAL "")
    set(IDF_PATH "$ENV{IDF_PATH}" CACHE INTERNAL "Copied from environment IDF_PATH")
    message(STATUS "IDF_PATH: ${IDF_PATH}")
endif()
if (NOT "$ENV{TARGET}" STREQUAL "")
    set(TARGET "$ENV{TARGET}" CACHE INTERNAL "Copied from environment TARGET")
    message(STATUS "TARGET: ${TARGET}")
endif()
set(CMAKE_TOOLCHAIN_FILE ${IDF_PATH}/tools/cmake/toolchain-${TARGET}.cmake CACHE INTERNAL "assembled CMAKE_TOOLCHAIN_FILE from ENV")
message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")


project(arduplane)
enable_language(C CXX ASM)

set(targets "esp32"  "esp32s3" )

if("${TARGET}" IN_LIST targets)
    # Include for ESP-IDF build system functions
    include($ENV{IDF_PATH}/tools/cmake/idf.cmake)
    # Create idf::{target} and idf::freertos static libraries

    #list of components in a normal s3 build:
    # bootloader bootloader_support efuse esp32s3 esp_common esp_hw_support esp_rom esp_system esptool_py freertos hal log main micro-ecc newlib partition_table soc spi_flash xtensa
    idf_build_process("${TARGET}"
                    # try and trim the build; additional components
                    # will be included as needed based on dependency tree
                    #
                    # although esptool_py does not generate static library,
                    # processing the component is needed for flashing related
                    # targets and file generation
                    # WARNING leaving (eg) fatfs out here causes libraries/AP_Filesystem/AP_Filesystem_ESP32.h , which includes #include "esp_vfs_fat.h" to not be able to find the right symbols
                    COMPONENTS  
                        bootloader bootloader_support efuse 
                        "${TARGET}"  #esp32s3
                        esp_common esp_hw_support esp_rom esp_system 
                        freertos
                        hal log 
                        #esp_rom             # for asprintf
                        newlib               # for time.h?
                        #micro-ecc    #Failed to resolve component 'micro-ecc'.
                        partition_table soc
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



include(./ardudefinitions.cmake)

#enable debugging symbols
target_compile_options(${elf_file} PRIVATE -g)
