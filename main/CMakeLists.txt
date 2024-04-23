#${BUILDDIR} and  ${BOARD_NAME} are 'global vars' set in the top-level CMakeLists.txt
string(CONCAT GCSMAVLINK "../" ${BUILDDIR} "/" ${BOARD_NAME} "/libraries/GCS_MAVLink")
message(STATUS "GCSMAVLINK: ${GCSMAVLINK}")

idf_component_register(SRCS "main.cpp" 
                    INCLUDE_DIRS ${GCSMAVLINK} "." ".." "../.." "../../.." "../libraries"  "../ArduPlane" "../build2/esp32s3empty/libraries/GCS_MAVLink" "../libraries/AP_HAL" "../libraries/AP_HAL_ESP32" "../libraries/AP_HAL_ESP32/boards/" "../../libraries/AP_HAL_ESP32/boards/"  
                    WHOLE_ARCHIVE
                    REQUIRES ARDULIBS 
                    )

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-reorder -Wno-trigraphs -Wno-unknown-pragmas)

add_definitions(-I./${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir
add_definitions(-I../../../../../../../${BUILDDIR}/${BOARD_NAME}/libraries/GCS_MAVLink) # generated mavlink code in build dir


# this alows ardupilot to call stuff like f_getfree in the fatfs library
add_definitions(-I./modules/esp_idf/components/fatfs/diskio)
add_definitions(-I./modules/esp_idf/components/fatfs/vfs)
add_definitions(-I./modules/esp_idf/components/fatfs/src)
add_definitions(-I../modules/esp_idf/components/fatfs/diskio)
add_definitions(-I../modules/esp_idf/components/fatfs/vfs)
add_definitions(-I../modules/esp_idf/components/fatfs/src)
add_definitions(-I../../modules/esp_idf/components/fatfs/diskio)
add_definitions(-I../../modules/esp_idf/components/fatfs/vfs)
add_definitions(-I../../modules/esp_idf/components/fatfs/src)
add_definitions(-I../../../modules/esp_idf/components/fatfs/diskio)
add_definitions(-I../../../modules/esp_idf/components/fatfs/vfs)
add_definitions(-I../../../modules/esp_idf/components/fatfs/src)
add_definitions(-I../../../../modules/esp_idf/components/fatfs/diskio)
add_definitions(-I../../../../modules/esp_idf/components/fatfs/vfs)
add_definitions(-I../../../../modules/esp_idf/components/fatfs/src)
add_definitions(-I../../../../../modules/esp_idf/components/fatfs/diskio)
add_definitions(-I../../../../../modules/esp_idf/components/fatfs/vfs)
add_definitions(-I../../../../../modules/esp_idf/components/fatfs/src)

# this allows ardupilot to find "sdkconfig.h"
add_definitions(-I./config)
add_definitions(-I../config)
add_definitions(-I../../config)
add_definitions(-I../../../config)
add_definitions(-I../../../../config)
add_definitions(-I../../../../../config)

# this allows ardupilot to find "config/sdkconfig.h"
add_definitions(-I./)
add_definitions(-I../)
add_definitions(-I../../)
add_definitions(-I../../../)
add_definitions(-I../../../../)
add_definitions(-I../../../../../)

add_definitions(-DBUILT_WITH_CMAKE=1)
