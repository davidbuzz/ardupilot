
set(objs_AP_AHRS_AntennaTracker_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs_AP_AHRS_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_AHRS_AntennaTracker_INCLUDES})

add_library(objs_AP_AHRS_AntennaTracker ${objs_AP_AHRS_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_AHRS_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_AHRS_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_AHRS_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_AHRS_AntennaTracker littlefs) #clean

set(objs_AP_Airspeed_AntennaTracker_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs_AP_Airspeed_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Airspeed_AntennaTracker_INCLUDES})

add_library(objs_AP_Airspeed_AntennaTracker ${objs_AP_Airspeed_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Airspeed_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Airspeed_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Airspeed_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Airspeed_AntennaTracker littlefs) #clean

set(objs_AP_Baro_AntennaTracker_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs_AP_Baro_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Baro_AntennaTracker_INCLUDES})

add_library(objs_AP_Baro_AntennaTracker ${objs_AP_Baro_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Baro_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Baro_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Baro_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Baro_AntennaTracker littlefs) #clean

set(objs_AP_BattMonitor_AntennaTracker_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs_AP_BattMonitor_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_BattMonitor_AntennaTracker_INCLUDES})

add_library(objs_AP_BattMonitor_AntennaTracker ${objs_AP_BattMonitor_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_BattMonitor_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_BattMonitor_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_AntennaTracker littlefs) #clean

set(objs_AP_BoardConfig_AntennaTracker_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs_AP_BoardConfig_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_BoardConfig_AntennaTracker_INCLUDES})

add_library(objs_AP_BoardConfig_AntennaTracker ${objs_AP_BoardConfig_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_BoardConfig_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_BoardConfig_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_AntennaTracker littlefs) #clean

set(objs_AP_HAL_AntennaTracker_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs_AP_HAL_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_HAL_AntennaTracker_INCLUDES})

add_library(objs_AP_HAL_AntennaTracker ${objs_AP_HAL_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_HAL_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_HAL_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_HAL_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_HAL_AntennaTracker littlefs) #clean

set(objs_AP_InertialSensor_AntennaTracker_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs_AP_InertialSensor_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_InertialSensor_AntennaTracker_INCLUDES})

add_library(objs_AP_InertialSensor_AntennaTracker ${objs_AP_InertialSensor_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_InertialSensor_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_InertialSensor_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_AntennaTracker littlefs) #clean

set(objs_AP_Math_AntennaTracker_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs_AP_Math_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Math_AntennaTracker_INCLUDES})

add_library(objs_AP_Math_AntennaTracker ${objs_AP_Math_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Math_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Math_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Math_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Math_AntennaTracker littlefs) #clean

set(objs_AP_Mission_AntennaTracker_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs_AP_Mission_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Mission_AntennaTracker_INCLUDES})

add_library(objs_AP_Mission_AntennaTracker ${objs_AP_Mission_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Mission_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Mission_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Mission_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Mission_AntennaTracker littlefs) #clean

set(objs_AP_DAL_AntennaTracker_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs_AP_DAL_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_DAL_AntennaTracker_INCLUDES})

add_library(objs_AP_DAL_AntennaTracker ${objs_AP_DAL_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_DAL_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_DAL_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_DAL_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_DAL_AntennaTracker littlefs) #clean

set(objs_AP_NavEKF2_AntennaTracker_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs_AP_NavEKF2_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_NavEKF2_AntennaTracker_INCLUDES})

add_library(objs_AP_NavEKF2_AntennaTracker ${objs_AP_NavEKF2_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_NavEKF2_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_NavEKF2_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_AntennaTracker littlefs) #clean

set(objs_AP_NavEKF3_AntennaTracker_SOURCES
    ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_GyroBias.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_Logging.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_VehicleStatus.cpp
    ../libraries/AP_NavEKF3/AP_NavEKF3_core.cpp)

set(objs_AP_NavEKF3_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_NavEKF3_AntennaTracker_INCLUDES})

add_library(objs_AP_NavEKF3_AntennaTracker ${objs_AP_NavEKF3_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_NavEKF3_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_NavEKF3_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_AntennaTracker littlefs) #clean

set(objs_AP_Rally_AntennaTracker_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs_AP_Rally_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Rally_AntennaTracker_INCLUDES})

add_library(objs_AP_Rally_AntennaTracker ${objs_AP_Rally_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Rally_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Rally_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Rally_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Rally_AntennaTracker littlefs) #clean

set(objs_AP_Scheduler_AntennaTracker_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs_AP_Scheduler_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Scheduler_AntennaTracker_INCLUDES})

add_library(objs_AP_Scheduler_AntennaTracker ${objs_AP_Scheduler_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Scheduler_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Scheduler_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Scheduler_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Scheduler_AntennaTracker littlefs) #clean

set(objs_AP_Terrain_AntennaTracker_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs_AP_Terrain_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Terrain_AntennaTracker_INCLUDES})

add_library(objs_AP_Terrain_AntennaTracker ${objs_AP_Terrain_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Terrain_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Terrain_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Terrain_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Terrain_AntennaTracker littlefs) #clean

set(objs_AP_Vehicle_AntennaTracker_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs_AP_Vehicle_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Vehicle_AntennaTracker_INCLUDES})

add_library(objs_AP_Vehicle_AntennaTracker ${objs_AP_Vehicle_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Vehicle_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Vehicle_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Vehicle_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Vehicle_AntennaTracker littlefs) #clean

set(objs_AP_Logger_AntennaTracker_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs_AP_Logger_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Logger_AntennaTracker_INCLUDES})

add_library(objs_AP_Logger_AntennaTracker ${objs_AP_Logger_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Logger_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Logger_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Logger_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Logger_AntennaTracker littlefs) #clean

set(objs_Filter_AntennaTracker_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs_Filter_AntennaTracker_INCLUDES
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

include_directories(${objs_Filter_AntennaTracker_INCLUDES})

add_library(objs_Filter_AntennaTracker ${objs_Filter_AntennaTracker_SOURCES})
target_compile_definitions(objs_Filter_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_Filter_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_AntennaTracker mavlink) #clean
target_link_libraries(objs_Filter_AntennaTracker dronecan) #clean
target_link_libraries(objs_Filter_AntennaTracker littlefs) #clean

set(objs_GCS_MAVLink_AntennaTracker_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs_GCS_MAVLink_AntennaTracker_INCLUDES
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

include_directories(${objs_GCS_MAVLink_AntennaTracker_INCLUDES})

add_library(objs_GCS_MAVLink_AntennaTracker ${objs_GCS_MAVLink_AntennaTracker_SOURCES})
target_compile_definitions(objs_GCS_MAVLink_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_GCS_MAVLink_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_AntennaTracker mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_AntennaTracker dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_AntennaTracker littlefs) #clean

set(objs_RC_Channel_AntennaTracker_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs_RC_Channel_AntennaTracker_INCLUDES
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

include_directories(${objs_RC_Channel_AntennaTracker_INCLUDES})

add_library(objs_RC_Channel_AntennaTracker ${objs_RC_Channel_AntennaTracker_SOURCES})
target_compile_definitions(objs_RC_Channel_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_RC_Channel_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_AntennaTracker mavlink) #clean
target_link_libraries(objs_RC_Channel_AntennaTracker dronecan) #clean
target_link_libraries(objs_RC_Channel_AntennaTracker littlefs) #clean

set(objs_StorageManager_AntennaTracker_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs_StorageManager_AntennaTracker_INCLUDES
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

include_directories(${objs_StorageManager_AntennaTracker_INCLUDES})

add_library(objs_StorageManager_AntennaTracker ${objs_StorageManager_AntennaTracker_SOURCES})
target_compile_definitions(objs_StorageManager_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_StorageManager_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_AntennaTracker mavlink) #clean
target_link_libraries(objs_StorageManager_AntennaTracker dronecan) #clean
target_link_libraries(objs_StorageManager_AntennaTracker littlefs) #clean

set(objs_AP_Frsky_Telem_AntennaTracker_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs_AP_Frsky_Telem_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Frsky_Telem_AntennaTracker_INCLUDES})

add_library(objs_AP_Frsky_Telem_AntennaTracker ${objs_AP_Frsky_Telem_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Frsky_Telem_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Frsky_Telem_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_AntennaTracker littlefs) #clean

set(objs_AP_IBus_Telem_AntennaTracker_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs_AP_IBus_Telem_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_IBus_Telem_AntennaTracker_INCLUDES})

add_library(objs_AP_IBus_Telem_AntennaTracker ${objs_AP_IBus_Telem_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_IBus_Telem_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_IBus_Telem_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_AntennaTracker littlefs) #clean

set(objs_AP_Relay_AntennaTracker_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs_AP_Relay_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Relay_AntennaTracker_INCLUDES})

add_library(objs_AP_Relay_AntennaTracker ${objs_AP_Relay_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Relay_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Relay_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Relay_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Relay_AntennaTracker littlefs) #clean

set(objs_AP_RCProtocol_AntennaTracker_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs_AP_RCProtocol_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_RCProtocol_AntennaTracker_INCLUDES})

add_library(objs_AP_RCProtocol_AntennaTracker ${objs_AP_RCProtocol_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_RCProtocol_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_RCProtocol_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_AntennaTracker littlefs) #clean

set(objs_AP_BLHeli_AntennaTracker_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs_AP_BLHeli_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_BLHeli_AntennaTracker_INCLUDES})

add_library(objs_AP_BLHeli_AntennaTracker ${objs_AP_BLHeli_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_BLHeli_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_BLHeli_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_BLHeli_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_BLHeli_AntennaTracker littlefs) #clean

set(objs_AP_Proximity_AntennaTracker_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs_AP_Proximity_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Proximity_AntennaTracker_INCLUDES})

add_library(objs_AP_Proximity_AntennaTracker ${objs_AP_Proximity_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Proximity_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Proximity_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Proximity_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Proximity_AntennaTracker littlefs) #clean

set(objs_AC_Fence_AntennaTracker_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs_AC_Fence_AntennaTracker_INCLUDES
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

include_directories(${objs_AC_Fence_AntennaTracker_INCLUDES})

add_library(objs_AC_Fence_AntennaTracker ${objs_AC_Fence_AntennaTracker_SOURCES})
target_compile_definitions(objs_AC_Fence_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AC_Fence_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_AntennaTracker mavlink) #clean
target_link_libraries(objs_AC_Fence_AntennaTracker dronecan) #clean
target_link_libraries(objs_AC_Fence_AntennaTracker littlefs) #clean

set(objs_AC_Avoidance_AntennaTracker_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs_AC_Avoidance_AntennaTracker_INCLUDES
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

include_directories(${objs_AC_Avoidance_AntennaTracker_INCLUDES})

add_library(objs_AC_Avoidance_AntennaTracker ${objs_AC_Avoidance_AntennaTracker_SOURCES})
target_compile_definitions(objs_AC_Avoidance_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AC_Avoidance_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_AntennaTracker mavlink) #clean
target_link_libraries(objs_AC_Avoidance_AntennaTracker dronecan) #clean
target_link_libraries(objs_AC_Avoidance_AntennaTracker littlefs) #clean

set(objs_AP_LandingGear_AntennaTracker_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs_AP_LandingGear_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_LandingGear_AntennaTracker_INCLUDES})

add_library(objs_AP_LandingGear_AntennaTracker ${objs_AP_LandingGear_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_LandingGear_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_LandingGear_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_LandingGear_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_LandingGear_AntennaTracker littlefs) #clean

set(objs_AP_OSD_AntennaTracker_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs_AP_OSD_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_OSD_AntennaTracker_INCLUDES})

add_library(objs_AP_OSD_AntennaTracker ${objs_AP_OSD_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_OSD_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_OSD_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_OSD_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_OSD_AntennaTracker littlefs) #clean

set(objs_AP_Filesystem_AntennaTracker_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs_AP_Filesystem_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Filesystem_AntennaTracker_INCLUDES})

add_library(objs_AP_Filesystem_AntennaTracker ${objs_AP_Filesystem_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Filesystem_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Filesystem_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Filesystem_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Filesystem_AntennaTracker littlefs) #clean

set(objs_AP_ADSB_AntennaTracker_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs_AP_ADSB_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_ADSB_AntennaTracker_INCLUDES})

add_library(objs_AP_ADSB_AntennaTracker ${objs_AP_ADSB_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_ADSB_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_ADSB_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_ADSB_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_ADSB_AntennaTracker littlefs) #clean

set(objs_AP_ADSB_sagetech-sdk_SOURCES
    ../libraries/AP_ADSB/sagetech-sdk/appendChecksum.c
    ../libraries/AP_ADSB/sagetech-sdk/calcChecksum.c
    ../libraries/AP_ADSB/sagetech-sdk/charArray2Buf.c
    ../libraries/AP_ADSB/sagetech-sdk/float2Buf.c
    ../libraries/AP_ADSB/sagetech-sdk/icao2Buf.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeAck.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeFlightId.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeInstall.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeMSR.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeSVR.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeDataReq.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeFlightId.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeGPS.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeInstall.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeOperating.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeTargetReq.c
    ../libraries/AP_ADSB/sagetech-sdk/target.c
    ../libraries/AP_ADSB/sagetech-sdk/toAlt.c
    ../libraries/AP_ADSB/sagetech-sdk/toGS.c
    ../libraries/AP_ADSB/sagetech-sdk/toHeading.c
    ../libraries/AP_ADSB/sagetech-sdk/toHeading2.c
    ../libraries/AP_ADSB/sagetech-sdk/toIcao.c
    ../libraries/AP_ADSB/sagetech-sdk/toInt16.c
    ../libraries/AP_ADSB/sagetech-sdk/toInt32.c
    ../libraries/AP_ADSB/sagetech-sdk/toLatLon.c
    ../libraries/AP_ADSB/sagetech-sdk/toTOA.c
    ../libraries/AP_ADSB/sagetech-sdk/toUint16.c
    ../libraries/AP_ADSB/sagetech-sdk/toUint32.c
    ../libraries/AP_ADSB/sagetech-sdk/toVel.c
    ../libraries/AP_ADSB/sagetech-sdk/uint162Buf.c
    ../libraries/AP_ADSB/sagetech-sdk/uint322Buf.c)

set(objs_AP_ADSB_sagetech-sdk_INCLUDES
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

include_directories(${objs_AP_ADSB_sagetech-sdk_INCLUDES})

add_library(objs_AP_ADSB_sagetech-sdk ${objs_AP_ADSB_sagetech-sdk_SOURCES})

set_target_properties(objs_AP_ADSB_sagetech-sdk PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_sagetech-sdk mavlink) #clean
target_link_libraries(objs_AP_ADSB_sagetech-sdk dronecan) #clean
target_link_libraries(objs_AP_ADSB_sagetech-sdk littlefs) #clean

set(objs_AP_ESC_Telem_AntennaTracker_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs_AP_ESC_Telem_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_ESC_Telem_AntennaTracker_INCLUDES})

add_library(objs_AP_ESC_Telem_AntennaTracker ${objs_AP_ESC_Telem_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_ESC_Telem_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_ESC_Telem_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_AntennaTracker littlefs) #clean

set(objs_AP_GyroFFT_AntennaTracker_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs_AP_GyroFFT_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_GyroFFT_AntennaTracker_INCLUDES})

add_library(objs_AP_GyroFFT_AntennaTracker ${objs_AP_GyroFFT_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_GyroFFT_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_GyroFFT_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_AntennaTracker littlefs) #clean

set(objs_AP_RCTelemetry_AntennaTracker_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs_AP_RCTelemetry_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_RCTelemetry_AntennaTracker_INCLUDES})

add_library(objs_AP_RCTelemetry_AntennaTracker ${objs_AP_RCTelemetry_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_RCTelemetry_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_RCTelemetry_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_AntennaTracker littlefs) #clean

set(objs_AP_ExternalAHRS_AntennaTracker_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs_AP_ExternalAHRS_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_ExternalAHRS_AntennaTracker_INCLUDES})

add_library(objs_AP_ExternalAHRS_AntennaTracker ${objs_AP_ExternalAHRS_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_ExternalAHRS_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_ExternalAHRS_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_AntennaTracker littlefs) #clean

set(objs_AP_TemperatureSensor_AntennaTracker_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs_AP_TemperatureSensor_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_TemperatureSensor_AntennaTracker_INCLUDES})

add_library(objs_AP_TemperatureSensor_AntennaTracker ${objs_AP_TemperatureSensor_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_TemperatureSensor_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_TemperatureSensor_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_AntennaTracker littlefs) #clean

set(objs_AP_AIS_AntennaTracker_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs_AP_AIS_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_AIS_AntennaTracker_INCLUDES})

add_library(objs_AP_AIS_AntennaTracker ${objs_AP_AIS_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_AIS_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_AIS_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_AIS_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_AIS_AntennaTracker littlefs) #clean

set(objs_AP_Arming_AntennaTracker_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs_AP_Arming_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Arming_AntennaTracker_INCLUDES})

add_library(objs_AP_Arming_AntennaTracker ${objs_AP_Arming_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Arming_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Arming_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Arming_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Arming_AntennaTracker littlefs) #clean

set(objs_AP_Follow_AntennaTracker_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs_AP_Follow_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Follow_AntennaTracker_INCLUDES})

add_library(objs_AP_Follow_AntennaTracker ${objs_AP_Follow_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Follow_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Follow_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Follow_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Follow_AntennaTracker littlefs) #clean

set(objs_AP_HAL_SITL_AntennaTracker_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs_AP_HAL_SITL_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_HAL_SITL_AntennaTracker_INCLUDES})

add_library(objs_AP_HAL_SITL_AntennaTracker ${objs_AP_HAL_SITL_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_HAL_SITL_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_HAL_SITL_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_AntennaTracker littlefs) #clean

set(objs_SITL_AntennaTracker_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs_SITL_AntennaTracker_INCLUDES
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

include_directories(${objs_SITL_AntennaTracker_INCLUDES})

add_library(objs_SITL_AntennaTracker ${objs_SITL_AntennaTracker_SOURCES})
target_compile_definitions(objs_SITL_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_SITL_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_AntennaTracker mavlink) #clean
target_link_libraries(objs_SITL_AntennaTracker dronecan) #clean
target_link_libraries(objs_SITL_AntennaTracker littlefs) #clean

set(objs_AP_Scripting_AntennaTracker_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs_AP_Scripting_AntennaTracker_INCLUDES
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

include_directories(${objs_AP_Scripting_AntennaTracker_INCLUDES})

add_library(objs_AP_Scripting_AntennaTracker ${objs_AP_Scripting_AntennaTracker_SOURCES})
target_compile_definitions(objs_AP_Scripting_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2

set_target_properties(objs_AP_Scripting_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Scripting_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Scripting_AntennaTracker littlefs) #clean

set(AntennaTracker_libs_SOURCES
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryCells.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryContinuous.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryInfoAux.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryPeriodic.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryTag.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.proximity_sensor.Proximity.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.trafficmonitor.TrafficReport.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.Heading.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.MovingBaselineData.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.RelPosHeading.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.Status.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.Button.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.NotifyState.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.SafetyState.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hex.equipment.flow.Measurement.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.himark.servo.ServoCmd.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.himark.servo.ServoInfo.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetEscID.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetMaintenanceInformation_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetMaintenanceInformation_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetMajorConfig_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetMajorConfig_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.RawCommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SelfTest_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SelfTest_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetAngle_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetAngle_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetBaud_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetBaud_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetDirection_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetDirection_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetID_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetID_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetLED_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetLED_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetReportingFrequency_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetReportingFrequency_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetThrottleSource_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetThrottleSource_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.StatusMsg1.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.StatusMsg2.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.StatusMsg3.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.FocCtrl.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.PUSHCAN.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.PUSHSCI.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.ParamCfg.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.ParamGet.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.volz.servo.ActuatorStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.CopterAttStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.GimbalAttitudeStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.GimbalControlData.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.GnssStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.GnssStatusReq.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/cuav.equipment.power.CBAT.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.protocol.CanStats.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.protocol.FlexDebug.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.protocol.GlobalTime.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.protocol.Stats.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.ArmStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.BasicID.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.Location.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.OperatorID.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.SecureCommand_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.SecureCommand_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.SelfID.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.System.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.hygrometer.Hygrometer.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.rc.RCInput.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.rpm.RPM.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.OutputEnable_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.OutputEnable_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.Stream.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.CoarseOrientation.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.Timestamp.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.ArrayCommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.Command.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.Status.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.MagneticFieldStrength.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.MagneticFieldStrength2.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.RawIMU.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.Solution.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.AngleOfAttack.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.IndicatedAirspeed.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.RawAirData.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.Sideslip.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.StaticPressure.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.StaticTemperature.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.TrueAirspeed.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.AngularCommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.GEOPOICommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.Mode.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.Status.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.device.Temperature.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.RPMCommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.RawCommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.Status.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.StatusExtended.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Auxiliary.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.ECEFPositionVelocity.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Fix.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Fix2.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.RTCMStream.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.hardpoint.Command.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.hardpoint.Status.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.FuelTankStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.reciprocating.CylinderStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.reciprocating.Status.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.BeepCommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.LightsCommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.RGB565.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.SingleLightCommand.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.BatteryInfo.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.CircuitStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.PrimaryPowerSupplyStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.range_sensor.Measurement.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.safety.ArmingStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.navigation.GlobalNavigationSolution.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.AccessCommandShell_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.AccessCommandShell_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.CANIfaceStats.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.DataTypeKind.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetDataTypeInfo_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetDataTypeInfo_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetNodeInfo_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetNodeInfo_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetTransportStats_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetTransportStats_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GlobalTimeSync.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.HardwareVersion.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.NodeStatus.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.Panic.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.RestartNode_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.RestartNode_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.SoftwareVersion.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.KeyValue.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.LogLevel.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.LogMessage.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.Allocation.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.AppendEntries_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.AppendEntries_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.Discovery.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.Entry.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.RequestVote_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.RequestVote_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Begin_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Begin_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Indication.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Delete_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Delete_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.EntryType.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Error.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetDirectoryEntryInfo_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetDirectoryEntryInfo_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetInfo_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetInfo_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Path.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Read_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Read_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Write_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Write_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.Empty.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.ExecuteOpcode_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.ExecuteOpcode_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.GetSet_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.GetSet_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.NumericValue.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.Value.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Broadcast.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Call_req.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Call_res.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Protocol.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.SerialConfig.c
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Targetted.c)

set(AntennaTracker_libs_INCLUDES
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

include_directories(${AntennaTracker_libs_INCLUDES})

add_library(AntennaTracker_libs ${AntennaTracker_libs_SOURCES})

set_target_properties(AntennaTracker_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AntennaTracker_libs libobjs_AP_AccelCal.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_ADC.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_AHRS.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Airspeed.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Baro.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_BattMonitor.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_BoardConfig.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Camera.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Common.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Compass.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Declination.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_GPS.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_GSOF.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_HAL.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_HAL_Empty.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_DDS.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_InertialSensor.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Math.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Mission.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_DAL.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_NavEKF.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_NavEKF2.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_NavEKF3.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Notify.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_OpticalFlow.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Param.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Rally.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_LightWareSerial.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RangeFinder.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Scheduler.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_SerialManager.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Terrain.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Vehicle.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_InternalError.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Logger.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_Filter.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_GCS_MAVLink.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_RC_Channel.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_SRV_Channel.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_StorageManager.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Tuning.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RPM.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RSSI.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Mount.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Module.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Button.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_ICEngine.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Networking.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Frsky_Telem.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_IBus_Telem.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_FlashStorage.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Relay.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_ServoRelayEvents.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Volz_Protocol.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_SBusOut.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_IOMCU.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Parachute.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RAMTRON.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RCProtocol.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Radio.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_TempCalibration.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_VisualOdom.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_BLHeli.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_ROMFS.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Proximity.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Gripper.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RTC.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AC_Sprayer.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AC_Fence.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AC_Avoidance.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_LandingGear.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RobotisServo.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_NMEA_Output.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_OSD.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Filesystem.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_ADSB.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AC_PID.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_SerialLED.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_EFI.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Hott_Telem.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_ESC_Telem.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Servo_Telem.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Stats.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_GyroFFT.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RCTelemetry.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Generator.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_MSP.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_OLC.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_WheelEncoder.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_ExternalAHRS.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_VideoTX.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_FETtecOneWire.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_TemperatureSensor.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Torqeedo.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_CustomRotations.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_AIS.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_OpenDroneID.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_CheckFirmware.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_ExternalControl.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_JSON.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Beacon.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Arming.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_RCMapper.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_MultiHeap.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Follow.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_CANManager.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_KDECAN.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_PiccoloCAN.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_DroneCAN.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_HAL_SITL.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_CSVReader.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_SITL.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AP_Scripting.a) #stripped
target_link_libraries(AntennaTracker_libs libobjs_AC_CustomControl.a) #stripped

set(bin_antennatracker_SOURCES
    GCS_MAVLink_Tracker.cpp
    GCS_Tracker.cpp
    Log.cpp
    Parameters.cpp
    RC_Channel_Tracker.cpp
    Tracker.cpp
    mode.cpp
    mode_auto.cpp
    mode_guided.cpp
    mode_manual.cpp
    mode_scan.cpp
    mode_servotest.cpp
    radio.cpp
    sensors.cpp
    servos.cpp
    system.cpp
    tracking.cpp)

set(bin_antennatracker_INCLUDES
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

include_directories(${bin_antennatracker_INCLUDES})

add_executable(bin_antennatracker ${bin_antennatracker_SOURCES})
target_compile_definitions(bin_antennatracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2


target_link_libraries(bin_antennatracker AntennaTracker_libs) #clean

