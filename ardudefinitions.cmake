# as close to waf as possible:
add_definitions(-fcheck-new)
add_definitions(-fsingle-precision-constant)
add_definitions(-Wno-psabi)
add_definitions(-std=gnu++11) #warning: command line option '-std=gnu++11' is valid for C++/ObjC++ but not for C
add_definitions(-fdata-sections)
add_definitions(-ffunction-sections)
add_definitions(-fno-exceptions)
add_definitions(-fsigned-char)
add_definitions(-nostdlib)
add_definitions(-Wno-reorder) # error: 'AP_HAL::HAL::serial_array' will be initialized after [-Werror=reorder]
add_definitions(-Wno-trigraphs)  #warning: trigraph ??- ignored, use -trigraphs to enable [-Wtrigraphs]

add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_ESP32)
add_definitions(-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_ESP32_S3EMPTY) 
add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_${VEHICLETYPE})
add_definitions(-DARDUPILOT_BUILD)
add_definitions(-D__AP_LINE__=__LINE__)
add_definitions(-DCYGWIN_BUILD)
add_definitions(-DAP_CUSTOMCONTROL_ENABLED=0)
add_definitions(-DAP_DDS_ENABLED=0)
add_definitions(-DAP_SCRIPTING_CHECKS=1)
add_definitions(-DENABLE_HEAP=0)
add_definitions(-DENABLE_ONVIF=0)
add_definitions(-DHAL_DEBUG_BUILD=1)
add_definitions(-DHAL_HAVE_HARDWARE_DOUBLE=1)
add_definitions(-DLUA_32BITS=1)
add_definitions(-DBUILT_WITH_CMAKE=1)
add_definitions(-D__GNU_VISIBLE) #https://www.esp32.com/viewtopic.php?t=5799
#add_definitions(-std=c++11) # for _GNU_SOURCE / asprintf as part of stdio.h no-good
add_definitions(-DAP_BUILD_TARGET_NAME=${VEHICLETYPE})
add_definitions(-DAP_GPS_ENABLED=1)
add_definitions(-D__XTENSA__=1)
# if (BOARD_NAME STREQUAL "sitl")     add_definitions(-DAP_SIM_ENABLED=1) endif()
add_definitions(-I.)
add_definitions(-I..)
add_definitions(-I../..)
add_definitions(-I../../..)

add_definitions(-I../${BUILDDIR}/config)     # places to look for the main sdkconfig
add_definitions(-I../../${BUILDDIR}/config)  # to find generated sdkconfig.h

add_definitions(-I../config)     # for the vehicle to compile, compiled second
add_definitions(-I../../config)  # for the libs to compile, compiled first

add_definitions(-I../)    # for the vehicle to compile, compiled second
add_definitions(-I../../) # for the libs to compile, compiled first

add_definitions(-Ilibraries)
add_definitions(-I../../libraries)

add_definitions(-Ilibraries/GCS_MAVLink)

add_definitions(-I../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink)  # generated mavlink code in build directory # for the libs to compile, compiled first
add_definitions(-I../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink)                                                 # for the vehicle to compile, compiled second

add_definitions(-I../../${VEHICLETYPE})  # for the libs to compile, compiled first
add_definitions(-I../${VEHICLETYPE})     # for the vehicle to compile, compiled second

add_definitions(-I../libraries/AP_Common/missing)     # for the vehicle to compile, compiled second
add_definitions(-I../../libraries/AP_Common/missing)  # for the libs to compile, compiled first

add_definitions(-I../../libraries/AP_HAL_ESP32)  # for stdio.h/asprintf etc
add_definitions(-I../../libraries/AP_HAL_ESP32/boards)  
    
# this is a kinda generated list from looking at the -Ixxx flags used by another (waf) esp32s3 build and documenting many of them
#  it shouldn't change too often
add_definitions(-I../../modules/esp_idf/components/xtensa/include)                               # for xtensa/hal.h
add_definitions(-I../../modules/esp_idf/components/xtensa/esp32s3/include)                       # for xtensa/config/core.h
add_definitions(-I../../modules/esp_idf/components/newlib/platform_include)                      # for esp_newlib.h
add_definitions(-I../../modules/esp_idf/components/freertos/include)                             # for freertos/FreeRTOS.h
add_definitions(-I../../modules/esp_idf/components/freertos/include/esp_additions/freertos)      # for FreeRTOSConfig.h
add_definitions(-I../../modules/esp_idf/components/freertos/port/xtensa/include)                 # for freertos/FreeRTOSConfig_arch.h
add_definitions(-I../../modules/esp_idf/components/freertos/include/esp_additions)               # for freertos/FreeRTOSConfig.h
add_definitions(-I../../modules/esp_idf/components/esp_hw_support/include)                       # for esp_intr_alloc.h
add_definitions(-I../../modules/esp_idf/components/esp_hw_support/include/soc)                   # for esp32s3/dport_access.h
add_definitions(-I../../modules/esp_idf/components/esp_hw_support/include/soc/esp32s3)
add_definitions(-I../../modules/esp_idf/components/esp_hw_support/port/esp32s3)
add_definitions(-I../../modules/esp_idf/components/esp_hw_support/port/esp32s3/private_include)
add_definitions(-I../../modules/esp_idf/components/heap/include)                                 # for esp_heap_caps.h
add_definitions(-I../../modules/esp_idf/components/log/include)                                  # for esp_log.h in AP_Filesystem_ESP32.h
add_definitions(-I../../modules/esp_idf/components/lwip/include/apps)                # for dhcpserver/dhcpserver.h
# add_definitions(-I../../modules/esp_idf/components/lwip/include/apps/sntp)
add_definitions(-I../../modules/esp_idf/components/lwip/lwip/src/include)            # for lwip/sockets.h in AP_HAL_ESP32/WiFiUdpDriver.h
add_definitions(-I../../modules/esp_idf/components/lwip/lwip/src/include/lwip)
add_definitions(-I../../modules/esp_idf/components/lwip/port/esp32/include)          # for lwipopts.h
add_definitions(-I../../modules/esp_idf/components/lwip/port/esp32/include/arch)
add_definitions(-I../../modules/esp_idf/components/soc/include)                      # for soc/gpio_periph.h
add_definitions(-I../../modules/esp_idf/components/soc/esp32s3)
add_definitions(-I../../modules/esp_idf/components/soc/esp32s3/include)              # for soc/io_mux_reg.h
add_definitions(-I../../modules/esp_idf/components/hal/esp32s3/include)              # for hal/cpu_ll.h
add_definitions(-I../../modules/esp_idf/components/hal/include)                      # for hal/gpio_types.h
add_definitions(-I../../modules/esp_idf/components/hal/platform_port/include)
add_definitions(-I../../modules/esp_idf/components/esp_rom/include)                  # for esp_rom_sys.h
add_definitions(-I../../modules/esp_idf/components/esp_rom/include/esp32s3)          # for rom/ets_sys.h
add_definitions(-I../../modules/esp_idf/components/esp_rom/esp32s3)
add_definitions(-I../../modules/esp_idf/components/esp_common/include)                # for esp_assert.h
add_definitions(-I../../modules/esp_idf/components/esp_system/include)               # for esp_private/crosscore_int.h
add_definitions(-I../../modules/esp_idf/components/esp_system/port/soc)
add_definitions(-I../../modules/esp_idf/components/esp_system/port/public_compat)
add_definitions(-I../../modules/esp_idf/components/vfs/include)                      # for esp_vfs.h in AP_Filesystem_ESP32.cpp
add_definitions(-I../../modules/esp_idf/components/esp_wifi/include)                 # for esp_wifi_types.h
add_definitions(-I../../modules/esp_idf/components/esp_event/include)                # for esp_event.h
add_definitions(-I../../modules/esp_idf/components/esp_netif/include)                # for esp_netif.h
add_definitions(-I../../modules/esp_idf/components/esp_eth/include)                  # for esp_eth_netif_glue.h
add_definitions(-I../../modules/esp_idf/components/tcpip_adapter/include)            # for tcpip_adapter.h
add_definitions(-I../../modules/esp_idf/components/esp_phy/include)
add_definitions(-I../../modules/esp_idf/components/esp_phy/esp32s3/include)
add_definitions(-I../../modules/esp_idf/components/esp_ipc/include)
add_definitions(-I../../modules/esp_idf/components/app_trace/include)
add_definitions(-I../../modules/esp_idf/components/esp_timer/include)                # for esp_timer.h
add_definitions(-I../../modules/esp_idf/components/driver/include)                   # for driver/uart.h
add_definitions(-I../../modules/esp_idf/components/driver/esp32s3/include)
add_definitions(-I../../modules/esp_idf/components/esp_pm/include)                   # for esp_pm.h
add_definitions(-I../../modules/esp_idf/components/esp_ringbuf/include)              # for freertos/ringbuf.h
add_definitions(-I../../modules/esp_idf/components/efuse/include)
add_definitions(-I../../modules/esp_idf/components/efuse/esp32s3/include)
add_definitions(-I../../modules/esp_idf/components/fatfs/diskio)
add_definitions(-I../../modules/esp_idf/components/fatfs/vfs)
add_definitions(-I../../modules/esp_idf/components/fatfs/src)
add_definitions(-I../../modules/esp_idf/components/wear_levelling/include)           # for wear_levelling.h
add_definitions(-I../../modules/esp_idf/components/spi_flash/include)                # for esp_partition.h
add_definitions(-I../../modules/esp_idf/components/sdmmc/include)                    # foir sdmmc_cmd.h  in AP_Filesystem_ESP32.h
add_definitions(-I../../modules/esp_idf/components/esp_adc_cal/include)              # for esp_adc_cal.h in AP_HAL_ESP32/AnalogIn.cpp 
add_definitions(-I../../modules/esp_idf/components/nvs_flash/include)
add_definitions(-I../../modules/esp_idf/components/mbedtls/port/include)
add_definitions(-I../../modules/esp_idf/components/mbedtls/mbedtls/include)
add_definitions(-I../../modules/esp_idf/components/mbedtls/esp_crt_bundle/include)
add_definitions(-I../../modules/esp_idf/components/mbedtls/mbedtls/include)
add_definitions(-I../../modules/esp_idf/components/app_update/include)               # for esp_ota_ops.h
add_definitions(-I../../modules/esp_idf/components/bootloader_support/include)       # for esp_image_format.h
#add_definitions(-I../../modules/esp_idf/components/bootloader/include)

add_definitions(-I../../libraries/AP_Common)  # this needs to be AFTER the *newlib/platform_include* above so we get time.h from newlib *first*

add_definitions(-I../modules/esp_idf/components/fatfs/diskio)     # for the vehicle to compile, compiled second
add_definitions(-I../modules/esp_idf/components/fatfs/vfs)        # for the vehicle to compile, compiled second
add_definitions(-I../modules/esp_idf/components/fatfs/src)        # for the vehicle to compile, compiled second

add_definitions(-I../../modules/esp_idf/components/fatfs/diskio)  # for the libs to compile, compiled first
add_definitions(-I../../modules/esp_idf/components/fatfs/vfs)     # for the libs to compile, compiled first
add_definitions(-I../../modules/esp_idf/components/fatfs/src)     # for the libs to compile, compiled first
