
set(objs_AP_AHRS_ArduSub_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs_AP_AHRS_ArduSub_INCLUDES
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

include_directories(${objs_AP_AHRS_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_AHRS_ArduSub ${objs_AP_AHRS_ArduSub_SOURCES})
set_target_properties(objs_AP_AHRS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_ArduSub mavlink)
target_link_libraries(objs_AP_AHRS_ArduSub dronecan)
target_link_libraries(objs_AP_AHRS_ArduSub littlefs)

set(objs_AP_Airspeed_ArduSub_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs_AP_Airspeed_ArduSub_INCLUDES
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

include_directories(${objs_AP_Airspeed_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Airspeed_ArduSub ${objs_AP_Airspeed_ArduSub_SOURCES})
set_target_properties(objs_AP_Airspeed_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_ArduSub mavlink)
target_link_libraries(objs_AP_Airspeed_ArduSub dronecan)
target_link_libraries(objs_AP_Airspeed_ArduSub littlefs)

set(objs_AP_Baro_ArduSub_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs_AP_Baro_ArduSub_INCLUDES
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

include_directories(${objs_AP_Baro_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Baro_ArduSub ${objs_AP_Baro_ArduSub_SOURCES})
set_target_properties(objs_AP_Baro_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_ArduSub mavlink)
target_link_libraries(objs_AP_Baro_ArduSub dronecan)
target_link_libraries(objs_AP_Baro_ArduSub littlefs)

set(objs_AP_BattMonitor_ArduSub_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs_AP_BattMonitor_ArduSub_INCLUDES
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

include_directories(${objs_AP_BattMonitor_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_BattMonitor_ArduSub ${objs_AP_BattMonitor_ArduSub_SOURCES})
set_target_properties(objs_AP_BattMonitor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_ArduSub mavlink)
target_link_libraries(objs_AP_BattMonitor_ArduSub dronecan)
target_link_libraries(objs_AP_BattMonitor_ArduSub littlefs)

set(objs_AP_BoardConfig_ArduSub_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs_AP_BoardConfig_ArduSub_INCLUDES
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

include_directories(${objs_AP_BoardConfig_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_BoardConfig_ArduSub ${objs_AP_BoardConfig_ArduSub_SOURCES})
set_target_properties(objs_AP_BoardConfig_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_ArduSub mavlink)
target_link_libraries(objs_AP_BoardConfig_ArduSub dronecan)
target_link_libraries(objs_AP_BoardConfig_ArduSub littlefs)

set(objs_AP_HAL_ArduSub_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs_AP_HAL_ArduSub_INCLUDES
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

include_directories(${objs_AP_HAL_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_HAL_ArduSub ${objs_AP_HAL_ArduSub_SOURCES})
set_target_properties(objs_AP_HAL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_ArduSub mavlink)
target_link_libraries(objs_AP_HAL_ArduSub dronecan)
target_link_libraries(objs_AP_HAL_ArduSub littlefs)

set(objs_AP_InertialSensor_ArduSub_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs_AP_InertialSensor_ArduSub_INCLUDES
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

include_directories(${objs_AP_InertialSensor_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_InertialSensor_ArduSub ${objs_AP_InertialSensor_ArduSub_SOURCES})
set_target_properties(objs_AP_InertialSensor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_ArduSub mavlink)
target_link_libraries(objs_AP_InertialSensor_ArduSub dronecan)
target_link_libraries(objs_AP_InertialSensor_ArduSub littlefs)

set(objs_AP_Math_ArduSub_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs_AP_Math_ArduSub_INCLUDES
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

include_directories(${objs_AP_Math_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Math_ArduSub ${objs_AP_Math_ArduSub_SOURCES})
set_target_properties(objs_AP_Math_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_ArduSub mavlink)
target_link_libraries(objs_AP_Math_ArduSub dronecan)
target_link_libraries(objs_AP_Math_ArduSub littlefs)

set(objs_AP_Mission_ArduSub_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs_AP_Mission_ArduSub_INCLUDES
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

include_directories(${objs_AP_Mission_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Mission_ArduSub ${objs_AP_Mission_ArduSub_SOURCES})
set_target_properties(objs_AP_Mission_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_ArduSub mavlink)
target_link_libraries(objs_AP_Mission_ArduSub dronecan)
target_link_libraries(objs_AP_Mission_ArduSub littlefs)

set(objs_AP_DAL_ArduSub_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs_AP_DAL_ArduSub_INCLUDES
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

include_directories(${objs_AP_DAL_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_DAL_ArduSub ${objs_AP_DAL_ArduSub_SOURCES})
set_target_properties(objs_AP_DAL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_ArduSub mavlink)
target_link_libraries(objs_AP_DAL_ArduSub dronecan)
target_link_libraries(objs_AP_DAL_ArduSub littlefs)

set(objs_AP_NavEKF2_ArduSub_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs_AP_NavEKF2_ArduSub_INCLUDES
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

include_directories(${objs_AP_NavEKF2_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_NavEKF2_ArduSub ${objs_AP_NavEKF2_ArduSub_SOURCES})
set_target_properties(objs_AP_NavEKF2_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_ArduSub mavlink)
target_link_libraries(objs_AP_NavEKF2_ArduSub dronecan)
target_link_libraries(objs_AP_NavEKF2_ArduSub littlefs)

set(objs_AP_NavEKF3_ArduSub_SOURCES
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

set(objs_AP_NavEKF3_ArduSub_INCLUDES
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

include_directories(${objs_AP_NavEKF3_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_NavEKF3_ArduSub ${objs_AP_NavEKF3_ArduSub_SOURCES})
set_target_properties(objs_AP_NavEKF3_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_ArduSub mavlink)
target_link_libraries(objs_AP_NavEKF3_ArduSub dronecan)
target_link_libraries(objs_AP_NavEKF3_ArduSub littlefs)

set(objs_AP_Rally_ArduSub_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs_AP_Rally_ArduSub_INCLUDES
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

include_directories(${objs_AP_Rally_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Rally_ArduSub ${objs_AP_Rally_ArduSub_SOURCES})
set_target_properties(objs_AP_Rally_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_ArduSub mavlink)
target_link_libraries(objs_AP_Rally_ArduSub dronecan)
target_link_libraries(objs_AP_Rally_ArduSub littlefs)

set(objs_AP_Scheduler_ArduSub_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs_AP_Scheduler_ArduSub_INCLUDES
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

include_directories(${objs_AP_Scheduler_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Scheduler_ArduSub ${objs_AP_Scheduler_ArduSub_SOURCES})
set_target_properties(objs_AP_Scheduler_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_ArduSub mavlink)
target_link_libraries(objs_AP_Scheduler_ArduSub dronecan)
target_link_libraries(objs_AP_Scheduler_ArduSub littlefs)

set(objs_AP_Terrain_ArduSub_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs_AP_Terrain_ArduSub_INCLUDES
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

include_directories(${objs_AP_Terrain_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Terrain_ArduSub ${objs_AP_Terrain_ArduSub_SOURCES})
set_target_properties(objs_AP_Terrain_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_ArduSub mavlink)
target_link_libraries(objs_AP_Terrain_ArduSub dronecan)
target_link_libraries(objs_AP_Terrain_ArduSub littlefs)

set(objs_AP_Vehicle_ArduSub_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs_AP_Vehicle_ArduSub_INCLUDES
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

include_directories(${objs_AP_Vehicle_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Vehicle_ArduSub ${objs_AP_Vehicle_ArduSub_SOURCES})
set_target_properties(objs_AP_Vehicle_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_ArduSub mavlink)
target_link_libraries(objs_AP_Vehicle_ArduSub dronecan)
target_link_libraries(objs_AP_Vehicle_ArduSub littlefs)

set(objs_AP_Logger_ArduSub_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs_AP_Logger_ArduSub_INCLUDES
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

include_directories(${objs_AP_Logger_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Logger_ArduSub ${objs_AP_Logger_ArduSub_SOURCES})
set_target_properties(objs_AP_Logger_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_ArduSub mavlink)
target_link_libraries(objs_AP_Logger_ArduSub dronecan)
target_link_libraries(objs_AP_Logger_ArduSub littlefs)

set(objs_Filter_ArduSub_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs_Filter_ArduSub_INCLUDES
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

include_directories(${objs_Filter_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_Filter_ArduSub ${objs_Filter_ArduSub_SOURCES})
set_target_properties(objs_Filter_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_ArduSub mavlink)
target_link_libraries(objs_Filter_ArduSub dronecan)
target_link_libraries(objs_Filter_ArduSub littlefs)

set(objs_GCS_MAVLink_ArduSub_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs_GCS_MAVLink_ArduSub_INCLUDES
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

include_directories(${objs_GCS_MAVLink_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_GCS_MAVLink_ArduSub ${objs_GCS_MAVLink_ArduSub_SOURCES})
set_target_properties(objs_GCS_MAVLink_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_ArduSub mavlink)
target_link_libraries(objs_GCS_MAVLink_ArduSub dronecan)
target_link_libraries(objs_GCS_MAVLink_ArduSub littlefs)

set(objs_RC_Channel_ArduSub_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs_RC_Channel_ArduSub_INCLUDES
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

include_directories(${objs_RC_Channel_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_RC_Channel_ArduSub ${objs_RC_Channel_ArduSub_SOURCES})
set_target_properties(objs_RC_Channel_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_ArduSub mavlink)
target_link_libraries(objs_RC_Channel_ArduSub dronecan)
target_link_libraries(objs_RC_Channel_ArduSub littlefs)

set(objs_StorageManager_ArduSub_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs_StorageManager_ArduSub_INCLUDES
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

include_directories(${objs_StorageManager_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_StorageManager_ArduSub ${objs_StorageManager_ArduSub_SOURCES})
set_target_properties(objs_StorageManager_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_ArduSub mavlink)
target_link_libraries(objs_StorageManager_ArduSub dronecan)
target_link_libraries(objs_StorageManager_ArduSub littlefs)

set(objs_AP_Frsky_Telem_ArduSub_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs_AP_Frsky_Telem_ArduSub_INCLUDES
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

include_directories(${objs_AP_Frsky_Telem_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Frsky_Telem_ArduSub ${objs_AP_Frsky_Telem_ArduSub_SOURCES})
set_target_properties(objs_AP_Frsky_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_ArduSub mavlink)
target_link_libraries(objs_AP_Frsky_Telem_ArduSub dronecan)
target_link_libraries(objs_AP_Frsky_Telem_ArduSub littlefs)

set(objs_AP_IBus_Telem_ArduSub_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs_AP_IBus_Telem_ArduSub_INCLUDES
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

include_directories(${objs_AP_IBus_Telem_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_IBus_Telem_ArduSub ${objs_AP_IBus_Telem_ArduSub_SOURCES})
set_target_properties(objs_AP_IBus_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_ArduSub mavlink)
target_link_libraries(objs_AP_IBus_Telem_ArduSub dronecan)
target_link_libraries(objs_AP_IBus_Telem_ArduSub littlefs)

set(objs_AP_Relay_ArduSub_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs_AP_Relay_ArduSub_INCLUDES
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

include_directories(${objs_AP_Relay_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Relay_ArduSub ${objs_AP_Relay_ArduSub_SOURCES})
set_target_properties(objs_AP_Relay_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_ArduSub mavlink)
target_link_libraries(objs_AP_Relay_ArduSub dronecan)
target_link_libraries(objs_AP_Relay_ArduSub littlefs)

set(objs_AP_RCProtocol_ArduSub_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs_AP_RCProtocol_ArduSub_INCLUDES
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

include_directories(${objs_AP_RCProtocol_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_RCProtocol_ArduSub ${objs_AP_RCProtocol_ArduSub_SOURCES})
set_target_properties(objs_AP_RCProtocol_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_ArduSub mavlink)
target_link_libraries(objs_AP_RCProtocol_ArduSub dronecan)
target_link_libraries(objs_AP_RCProtocol_ArduSub littlefs)

set(objs_AP_BLHeli_ArduSub_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs_AP_BLHeli_ArduSub_INCLUDES
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

include_directories(${objs_AP_BLHeli_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_BLHeli_ArduSub ${objs_AP_BLHeli_ArduSub_SOURCES})
set_target_properties(objs_AP_BLHeli_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_ArduSub mavlink)
target_link_libraries(objs_AP_BLHeli_ArduSub dronecan)
target_link_libraries(objs_AP_BLHeli_ArduSub littlefs)

set(objs_AP_Proximity_ArduSub_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs_AP_Proximity_ArduSub_INCLUDES
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

include_directories(${objs_AP_Proximity_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Proximity_ArduSub ${objs_AP_Proximity_ArduSub_SOURCES})
set_target_properties(objs_AP_Proximity_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_ArduSub mavlink)
target_link_libraries(objs_AP_Proximity_ArduSub dronecan)
target_link_libraries(objs_AP_Proximity_ArduSub littlefs)

set(objs_AC_Fence_ArduSub_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs_AC_Fence_ArduSub_INCLUDES
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

include_directories(${objs_AC_Fence_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AC_Fence_ArduSub ${objs_AC_Fence_ArduSub_SOURCES})
set_target_properties(objs_AC_Fence_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_ArduSub mavlink)
target_link_libraries(objs_AC_Fence_ArduSub dronecan)
target_link_libraries(objs_AC_Fence_ArduSub littlefs)

set(objs_AC_Avoidance_ArduSub_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs_AC_Avoidance_ArduSub_INCLUDES
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

include_directories(${objs_AC_Avoidance_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AC_Avoidance_ArduSub ${objs_AC_Avoidance_ArduSub_SOURCES})
set_target_properties(objs_AC_Avoidance_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_ArduSub mavlink)
target_link_libraries(objs_AC_Avoidance_ArduSub dronecan)
target_link_libraries(objs_AC_Avoidance_ArduSub littlefs)

set(objs_AP_LandingGear_ArduSub_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs_AP_LandingGear_ArduSub_INCLUDES
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

include_directories(${objs_AP_LandingGear_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_LandingGear_ArduSub ${objs_AP_LandingGear_ArduSub_SOURCES})
set_target_properties(objs_AP_LandingGear_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_ArduSub mavlink)
target_link_libraries(objs_AP_LandingGear_ArduSub dronecan)
target_link_libraries(objs_AP_LandingGear_ArduSub littlefs)

set(objs_AP_OSD_ArduSub_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs_AP_OSD_ArduSub_INCLUDES
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

include_directories(${objs_AP_OSD_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_OSD_ArduSub ${objs_AP_OSD_ArduSub_SOURCES})
set_target_properties(objs_AP_OSD_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_ArduSub mavlink)
target_link_libraries(objs_AP_OSD_ArduSub dronecan)
target_link_libraries(objs_AP_OSD_ArduSub littlefs)

set(objs_AP_Filesystem_ArduSub_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs_AP_Filesystem_ArduSub_INCLUDES
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

include_directories(${objs_AP_Filesystem_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Filesystem_ArduSub ${objs_AP_Filesystem_ArduSub_SOURCES})
set_target_properties(objs_AP_Filesystem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_ArduSub mavlink)
target_link_libraries(objs_AP_Filesystem_ArduSub dronecan)
target_link_libraries(objs_AP_Filesystem_ArduSub littlefs)

set(objs_AP_ADSB_ArduSub_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs_AP_ADSB_ArduSub_INCLUDES
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

include_directories(${objs_AP_ADSB_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_ADSB_ArduSub ${objs_AP_ADSB_ArduSub_SOURCES})
set_target_properties(objs_AP_ADSB_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_ArduSub mavlink)
target_link_libraries(objs_AP_ADSB_ArduSub dronecan)
target_link_libraries(objs_AP_ADSB_ArduSub littlefs)

set(objs_AP_ESC_Telem_ArduSub_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs_AP_ESC_Telem_ArduSub_INCLUDES
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

include_directories(${objs_AP_ESC_Telem_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_ESC_Telem_ArduSub ${objs_AP_ESC_Telem_ArduSub_SOURCES})
set_target_properties(objs_AP_ESC_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_ArduSub mavlink)
target_link_libraries(objs_AP_ESC_Telem_ArduSub dronecan)
target_link_libraries(objs_AP_ESC_Telem_ArduSub littlefs)

set(objs_AP_GyroFFT_ArduSub_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs_AP_GyroFFT_ArduSub_INCLUDES
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

include_directories(${objs_AP_GyroFFT_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_GyroFFT_ArduSub ${objs_AP_GyroFFT_ArduSub_SOURCES})
set_target_properties(objs_AP_GyroFFT_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_ArduSub mavlink)
target_link_libraries(objs_AP_GyroFFT_ArduSub dronecan)
target_link_libraries(objs_AP_GyroFFT_ArduSub littlefs)

set(objs_AP_RCTelemetry_ArduSub_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs_AP_RCTelemetry_ArduSub_INCLUDES
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

include_directories(${objs_AP_RCTelemetry_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_RCTelemetry_ArduSub ${objs_AP_RCTelemetry_ArduSub_SOURCES})
set_target_properties(objs_AP_RCTelemetry_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_ArduSub mavlink)
target_link_libraries(objs_AP_RCTelemetry_ArduSub dronecan)
target_link_libraries(objs_AP_RCTelemetry_ArduSub littlefs)

set(objs_AP_ExternalAHRS_ArduSub_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs_AP_ExternalAHRS_ArduSub_INCLUDES
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

include_directories(${objs_AP_ExternalAHRS_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_ExternalAHRS_ArduSub ${objs_AP_ExternalAHRS_ArduSub_SOURCES})
set_target_properties(objs_AP_ExternalAHRS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_ArduSub mavlink)
target_link_libraries(objs_AP_ExternalAHRS_ArduSub dronecan)
target_link_libraries(objs_AP_ExternalAHRS_ArduSub littlefs)

set(objs_AP_TemperatureSensor_ArduSub_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs_AP_TemperatureSensor_ArduSub_INCLUDES
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

include_directories(${objs_AP_TemperatureSensor_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_TemperatureSensor_ArduSub ${objs_AP_TemperatureSensor_ArduSub_SOURCES})
set_target_properties(objs_AP_TemperatureSensor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_ArduSub mavlink)
target_link_libraries(objs_AP_TemperatureSensor_ArduSub dronecan)
target_link_libraries(objs_AP_TemperatureSensor_ArduSub littlefs)

set(objs_AP_AIS_ArduSub_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs_AP_AIS_ArduSub_INCLUDES
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

include_directories(${objs_AP_AIS_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_AIS_ArduSub ${objs_AP_AIS_ArduSub_SOURCES})
set_target_properties(objs_AP_AIS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_ArduSub mavlink)
target_link_libraries(objs_AP_AIS_ArduSub dronecan)
target_link_libraries(objs_AP_AIS_ArduSub littlefs)

set(objs_AP_Arming_ArduSub_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs_AP_Arming_ArduSub_INCLUDES
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

include_directories(${objs_AP_Arming_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Arming_ArduSub ${objs_AP_Arming_ArduSub_SOURCES})
set_target_properties(objs_AP_Arming_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_ArduSub mavlink)
target_link_libraries(objs_AP_Arming_ArduSub dronecan)
target_link_libraries(objs_AP_Arming_ArduSub littlefs)

set(objs_AP_Follow_ArduSub_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs_AP_Follow_ArduSub_INCLUDES
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

include_directories(${objs_AP_Follow_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Follow_ArduSub ${objs_AP_Follow_ArduSub_SOURCES})
set_target_properties(objs_AP_Follow_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_ArduSub mavlink)
target_link_libraries(objs_AP_Follow_ArduSub dronecan)
target_link_libraries(objs_AP_Follow_ArduSub littlefs)

set(objs_AC_AttitudeControl_ArduSub_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs_AC_AttitudeControl_ArduSub_INCLUDES
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

include_directories(${objs_AC_AttitudeControl_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AC_AttitudeControl_ArduSub ${objs_AC_AttitudeControl_ArduSub_SOURCES})
set_target_properties(objs_AC_AttitudeControl_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_ArduSub mavlink)
target_link_libraries(objs_AC_AttitudeControl_ArduSub dronecan)
target_link_libraries(objs_AC_AttitudeControl_ArduSub littlefs)

set(objs_AC_WPNav_ArduSub_SOURCES
    ../libraries/AC_WPNav/AC_Loiter.cpp)

set(objs_AC_WPNav_ArduSub_INCLUDES
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

include_directories(${objs_AC_WPNav_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AC_WPNav_ArduSub ${objs_AC_WPNav_ArduSub_SOURCES})
set_target_properties(objs_AC_WPNav_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav_ArduSub mavlink)
target_link_libraries(objs_AC_WPNav_ArduSub dronecan)
target_link_libraries(objs_AC_WPNav_ArduSub littlefs)

set(objs_AP_Motors_ArduSub_SOURCES
    ../libraries/AP_Motors/AP_MotorsMatrix.cpp
    ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    ../libraries/AP_Motors/AP_MotorsTri.cpp
    ../libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp)

set(objs_AP_Motors_ArduSub_INCLUDES
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

include_directories(${objs_AP_Motors_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Motors_ArduSub ${objs_AP_Motors_ArduSub_SOURCES})
set_target_properties(objs_AP_Motors_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors_ArduSub mavlink)
target_link_libraries(objs_AP_Motors_ArduSub dronecan)
target_link_libraries(objs_AP_Motors_ArduSub littlefs)

set(objs_AP_HAL_SITL_ArduSub_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs_AP_HAL_SITL_ArduSub_INCLUDES
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

include_directories(${objs_AP_HAL_SITL_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_HAL_SITL_ArduSub ${objs_AP_HAL_SITL_ArduSub_SOURCES})
set_target_properties(objs_AP_HAL_SITL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_ArduSub mavlink)
target_link_libraries(objs_AP_HAL_SITL_ArduSub dronecan)
target_link_libraries(objs_AP_HAL_SITL_ArduSub littlefs)

set(objs_SITL_ArduSub_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs_SITL_ArduSub_INCLUDES
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

include_directories(${objs_SITL_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_SITL_ArduSub ${objs_SITL_ArduSub_SOURCES})
set_target_properties(objs_SITL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_ArduSub mavlink)
target_link_libraries(objs_SITL_ArduSub dronecan)
target_link_libraries(objs_SITL_ArduSub littlefs)

set(objs_AP_Scripting_ArduSub_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs_AP_Scripting_ArduSub_INCLUDES
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

include_directories(${objs_AP_Scripting_ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs_AP_Scripting_ArduSub ${objs_AP_Scripting_ArduSub_SOURCES})
set_target_properties(objs_AP_Scripting_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_ArduSub mavlink)
target_link_libraries(objs_AP_Scripting_ArduSub dronecan)
target_link_libraries(objs_AP_Scripting_ArduSub littlefs)

set(ArduSub_libs_SOURCES
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

set(ArduSub_libs_INCLUDES
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

include_directories(${ArduSub_libs_INCLUDES})

add_library(ArduSub_libs ${ArduSub_libs_SOURCES})
set_target_properties(ArduSub_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(ArduSub_libs objs/AP_AccelCal)
target_link_libraries(ArduSub_libs objs/AP_AccelCal/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ADC)
target_link_libraries(ArduSub_libs objs/AP_ADC/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_AHRS)
target_link_libraries(ArduSub_libs objs/AP_AHRS/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Airspeed)
target_link_libraries(ArduSub_libs objs/AP_Airspeed/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Baro)
target_link_libraries(ArduSub_libs objs/AP_Baro/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_BattMonitor)
target_link_libraries(ArduSub_libs objs/AP_BattMonitor/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_BoardConfig)
target_link_libraries(ArduSub_libs objs/AP_BoardConfig/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Camera)
target_link_libraries(ArduSub_libs objs/AP_Camera/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Common)
target_link_libraries(ArduSub_libs objs/AP_Common/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Compass)
target_link_libraries(ArduSub_libs objs/AP_Compass/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Declination)
target_link_libraries(ArduSub_libs objs/AP_Declination/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_GPS)
target_link_libraries(ArduSub_libs objs/AP_GPS/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_GSOF)
target_link_libraries(ArduSub_libs objs/AP_GSOF/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_HAL)
target_link_libraries(ArduSub_libs objs/AP_HAL/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_HAL_Empty)
target_link_libraries(ArduSub_libs objs/AP_HAL_Empty/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_DDS)
target_link_libraries(ArduSub_libs objs/AP_DDS/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_InertialSensor)
target_link_libraries(ArduSub_libs objs/AP_InertialSensor/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Math)
target_link_libraries(ArduSub_libs objs/AP_Math/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Mission)
target_link_libraries(ArduSub_libs objs/AP_Mission/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_DAL)
target_link_libraries(ArduSub_libs objs/AP_DAL/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_NavEKF)
target_link_libraries(ArduSub_libs objs/AP_NavEKF/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_NavEKF2)
target_link_libraries(ArduSub_libs objs/AP_NavEKF2/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_NavEKF3)
target_link_libraries(ArduSub_libs objs/AP_NavEKF3/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Notify)
target_link_libraries(ArduSub_libs objs/AP_Notify/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_OpticalFlow)
target_link_libraries(ArduSub_libs objs/AP_OpticalFlow/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Param)
target_link_libraries(ArduSub_libs objs/AP_Param/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Rally)
target_link_libraries(ArduSub_libs objs/AP_Rally/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_LightWareSerial)
target_link_libraries(ArduSub_libs objs/AP_LightWareSerial/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RangeFinder)
target_link_libraries(ArduSub_libs objs/AP_RangeFinder/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Scheduler)
target_link_libraries(ArduSub_libs objs/AP_Scheduler/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_SerialManager)
target_link_libraries(ArduSub_libs objs/AP_SerialManager/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Terrain)
target_link_libraries(ArduSub_libs objs/AP_Terrain/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Vehicle)
target_link_libraries(ArduSub_libs objs/AP_Vehicle/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_InternalError)
target_link_libraries(ArduSub_libs objs/AP_InternalError/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Logger)
target_link_libraries(ArduSub_libs objs/AP_Logger/ArduSub)
target_link_libraries(ArduSub_libs objs/Filter)
target_link_libraries(ArduSub_libs objs/Filter/ArduSub)
target_link_libraries(ArduSub_libs objs/GCS_MAVLink)
target_link_libraries(ArduSub_libs objs/GCS_MAVLink/ArduSub)
target_link_libraries(ArduSub_libs objs/RC_Channel)
target_link_libraries(ArduSub_libs objs/RC_Channel/ArduSub)
target_link_libraries(ArduSub_libs objs/SRV_Channel)
target_link_libraries(ArduSub_libs objs/SRV_Channel/ArduSub)
target_link_libraries(ArduSub_libs objs/StorageManager)
target_link_libraries(ArduSub_libs objs/StorageManager/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Tuning)
target_link_libraries(ArduSub_libs objs/AP_Tuning/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RPM)
target_link_libraries(ArduSub_libs objs/AP_RPM/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RSSI)
target_link_libraries(ArduSub_libs objs/AP_RSSI/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Mount)
target_link_libraries(ArduSub_libs objs/AP_Mount/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Module)
target_link_libraries(ArduSub_libs objs/AP_Module/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Button)
target_link_libraries(ArduSub_libs objs/AP_Button/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ICEngine)
target_link_libraries(ArduSub_libs objs/AP_ICEngine/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Networking)
target_link_libraries(ArduSub_libs objs/AP_Networking/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Frsky_Telem)
target_link_libraries(ArduSub_libs objs/AP_Frsky_Telem/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_IBus_Telem)
target_link_libraries(ArduSub_libs objs/AP_IBus_Telem/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_FlashStorage)
target_link_libraries(ArduSub_libs objs/AP_FlashStorage/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Relay)
target_link_libraries(ArduSub_libs objs/AP_Relay/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ServoRelayEvents)
target_link_libraries(ArduSub_libs objs/AP_ServoRelayEvents/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Volz_Protocol)
target_link_libraries(ArduSub_libs objs/AP_Volz_Protocol/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_SBusOut)
target_link_libraries(ArduSub_libs objs/AP_SBusOut/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_IOMCU)
target_link_libraries(ArduSub_libs objs/AP_IOMCU/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Parachute)
target_link_libraries(ArduSub_libs objs/AP_Parachute/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RAMTRON)
target_link_libraries(ArduSub_libs objs/AP_RAMTRON/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RCProtocol)
target_link_libraries(ArduSub_libs objs/AP_RCProtocol/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Radio)
target_link_libraries(ArduSub_libs objs/AP_Radio/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_TempCalibration)
target_link_libraries(ArduSub_libs objs/AP_TempCalibration/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_VisualOdom)
target_link_libraries(ArduSub_libs objs/AP_VisualOdom/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_BLHeli)
target_link_libraries(ArduSub_libs objs/AP_BLHeli/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ROMFS)
target_link_libraries(ArduSub_libs objs/AP_ROMFS/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Proximity)
target_link_libraries(ArduSub_libs objs/AP_Proximity/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Gripper)
target_link_libraries(ArduSub_libs objs/AP_Gripper/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RTC)
target_link_libraries(ArduSub_libs objs/AP_RTC/ArduSub)
target_link_libraries(ArduSub_libs objs/AC_Sprayer)
target_link_libraries(ArduSub_libs objs/AC_Sprayer/ArduSub)
target_link_libraries(ArduSub_libs objs/AC_Fence)
target_link_libraries(ArduSub_libs objs/AC_Fence/ArduSub)
target_link_libraries(ArduSub_libs objs/AC_Avoidance)
target_link_libraries(ArduSub_libs objs/AC_Avoidance/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_LandingGear)
target_link_libraries(ArduSub_libs objs/AP_LandingGear/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RobotisServo)
target_link_libraries(ArduSub_libs objs/AP_RobotisServo/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_NMEA_Output)
target_link_libraries(ArduSub_libs objs/AP_NMEA_Output/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_OSD)
target_link_libraries(ArduSub_libs objs/AP_OSD/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Filesystem)
target_link_libraries(ArduSub_libs objs/AP_Filesystem/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ADSB)
target_link_libraries(ArduSub_libs objs/AP_ADSB/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ADSB/sagetech-sdk)
target_link_libraries(ArduSub_libs objs/AP_ADSB/sagetech-sdk/ArduSub)
target_link_libraries(ArduSub_libs objs/AC_PID)
target_link_libraries(ArduSub_libs objs/AC_PID/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_SerialLED)
target_link_libraries(ArduSub_libs objs/AP_SerialLED/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_EFI)
target_link_libraries(ArduSub_libs objs/AP_EFI/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Hott_Telem)
target_link_libraries(ArduSub_libs objs/AP_Hott_Telem/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ESC_Telem)
target_link_libraries(ArduSub_libs objs/AP_ESC_Telem/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Servo_Telem)
target_link_libraries(ArduSub_libs objs/AP_Servo_Telem/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Stats)
target_link_libraries(ArduSub_libs objs/AP_Stats/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_GyroFFT)
target_link_libraries(ArduSub_libs objs/AP_GyroFFT/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RCTelemetry)
target_link_libraries(ArduSub_libs objs/AP_RCTelemetry/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Generator)
target_link_libraries(ArduSub_libs objs/AP_Generator/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_MSP)
target_link_libraries(ArduSub_libs objs/AP_MSP/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_OLC)
target_link_libraries(ArduSub_libs objs/AP_OLC/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_WheelEncoder)
target_link_libraries(ArduSub_libs objs/AP_WheelEncoder/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ExternalAHRS)
target_link_libraries(ArduSub_libs objs/AP_ExternalAHRS/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_VideoTX)
target_link_libraries(ArduSub_libs objs/AP_VideoTX/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_FETtecOneWire)
target_link_libraries(ArduSub_libs objs/AP_FETtecOneWire/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_TemperatureSensor)
target_link_libraries(ArduSub_libs objs/AP_TemperatureSensor/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Torqeedo)
target_link_libraries(ArduSub_libs objs/AP_Torqeedo/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_CustomRotations)
target_link_libraries(ArduSub_libs objs/AP_CustomRotations/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_AIS)
target_link_libraries(ArduSub_libs objs/AP_AIS/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_OpenDroneID)
target_link_libraries(ArduSub_libs objs/AP_OpenDroneID/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_CheckFirmware)
target_link_libraries(ArduSub_libs objs/AP_CheckFirmware/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_ExternalControl)
target_link_libraries(ArduSub_libs objs/AP_ExternalControl/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_JSON)
target_link_libraries(ArduSub_libs objs/AP_JSON/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Beacon)
target_link_libraries(ArduSub_libs objs/AP_Beacon/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Arming)
target_link_libraries(ArduSub_libs objs/AP_Arming/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_RCMapper)
target_link_libraries(ArduSub_libs objs/AP_RCMapper/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_MultiHeap)
target_link_libraries(ArduSub_libs objs/AP_MultiHeap/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Follow)
target_link_libraries(ArduSub_libs objs/AP_Follow/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_CANManager)
target_link_libraries(ArduSub_libs objs/AP_CANManager/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_KDECAN)
target_link_libraries(ArduSub_libs objs/AP_KDECAN/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_PiccoloCAN)
target_link_libraries(ArduSub_libs objs/AP_PiccoloCAN/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_PiccoloCAN/piccolo_protocol)
target_link_libraries(ArduSub_libs objs/AP_PiccoloCAN/piccolo_protocol/ArduSub)
target_link_libraries(ArduSub_libs objs/AC_AttitudeControl)
target_link_libraries(ArduSub_libs objs/AC_AttitudeControl/ArduSub)
target_link_libraries(ArduSub_libs objs/AC_WPNav)
target_link_libraries(ArduSub_libs objs/AC_WPNav/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_InertialNav)
target_link_libraries(ArduSub_libs objs/AP_InertialNav/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_JSButton)
target_link_libraries(ArduSub_libs objs/AP_JSButton/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_LeakDetector)
target_link_libraries(ArduSub_libs objs/AP_LeakDetector/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Motors)
target_link_libraries(ArduSub_libs objs/AP_Motors/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_DroneCAN)
target_link_libraries(ArduSub_libs objs/AP_DroneCAN/ArduSub)
target_link_libraries(ArduSub_libs objs/modules/DroneCAN/libcanard/*.c)
target_link_libraries(ArduSub_libs objs/modules/DroneCAN/libcanard/*.c/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_HAL_SITL)
target_link_libraries(ArduSub_libs objs/AP_HAL_SITL/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_CSVReader)
target_link_libraries(ArduSub_libs objs/AP_CSVReader/ArduSub)
target_link_libraries(ArduSub_libs objs/SITL)
target_link_libraries(ArduSub_libs objs/SITL/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Scripting)
target_link_libraries(ArduSub_libs objs/AP_Scripting/ArduSub)
target_link_libraries(ArduSub_libs objs/AP_Scripting/lua/src)
target_link_libraries(ArduSub_libs objs/AP_Scripting/lua/src/ArduSub)
target_link_libraries(ArduSub_libs objs/AC_CustomControl)
target_link_libraries(ArduSub_libs objs/AC_CustomControl/ArduSub)

set(bin_ardusub_SOURCES
    AP_Arming_Sub.cpp
    AP_State.cpp
    Attitude.cpp
    GCS_MAVLink_Sub.cpp
    GCS_Sub.cpp
    Log.cpp
    Parameters.cpp
    RC_Channel_Sub.cpp
    Sub.cpp
    UserCode.cpp
    actuators.cpp
    commands.cpp
    commands_logic.cpp
    failsafe.cpp
    fence.cpp
    inertia.cpp
    joystick.cpp
    mode.cpp
    mode_acro.cpp
    mode_althold.cpp
    mode_auto.cpp
    mode_circle.cpp
    mode_guided.cpp
    mode_manual.cpp
    mode_motordetect.cpp
    mode_poshold.cpp
    mode_stabilize.cpp
    mode_surface.cpp
    mode_surftrak.cpp
    motors.cpp
    radio.cpp
    script_button.cpp
    sensors.cpp
    surface_bottom_detector.cpp
    system.cpp
    terrain.cpp
    turn_counter.cpp)

set(bin_ardusub_INCLUDES
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

include_directories(${bin_ardusub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_executable(bin_ardusub ${bin_ardusub_SOURCES})

target_link_libraries(bin_ardusub ArduSub_libs)

