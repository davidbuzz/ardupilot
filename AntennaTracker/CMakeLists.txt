
#------------------------------------------------

set(objs_AP_AHRS_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs_AP_AHRS_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_AHRS ${objs_AP_AHRS_SOURCES}) #2
target_compile_definitions(objs_AP_AHRS PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_AHRS PRIVATE ${objs_AP_AHRS_INCLUDES})

set_target_properties(objs_AP_AHRS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS mavlink) #clean
target_link_libraries(objs_AP_AHRS dronecan) #clean
target_link_libraries(objs_AP_AHRS littlefs) #clean

