
set(objs/AP_AHRS/AntennaTracker_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs/AP_AHRS/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_AHRS/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_AHRS/AntennaTracker ${objs/AP_AHRS/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_AHRS/AntennaTracker mavlink)
target_link_libraries(objs/AP_AHRS/AntennaTracker dronecan)
target_link_libraries(objs/AP_AHRS/AntennaTracker littlefs)

set(objs/AP_Airspeed/AntennaTracker_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs/AP_Airspeed/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Airspeed/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Airspeed/AntennaTracker ${objs/AP_Airspeed/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Airspeed/AntennaTracker mavlink)
target_link_libraries(objs/AP_Airspeed/AntennaTracker dronecan)
target_link_libraries(objs/AP_Airspeed/AntennaTracker littlefs)

set(objs/AP_Baro/AntennaTracker_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs/AP_Baro/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Baro/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Baro/AntennaTracker ${objs/AP_Baro/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Baro/AntennaTracker mavlink)
target_link_libraries(objs/AP_Baro/AntennaTracker dronecan)
target_link_libraries(objs/AP_Baro/AntennaTracker littlefs)

set(objs/AP_BattMonitor/AntennaTracker_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs/AP_BattMonitor/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_BattMonitor/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_BattMonitor/AntennaTracker ${objs/AP_BattMonitor/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_BattMonitor/AntennaTracker mavlink)
target_link_libraries(objs/AP_BattMonitor/AntennaTracker dronecan)
target_link_libraries(objs/AP_BattMonitor/AntennaTracker littlefs)

set(objs/AP_BoardConfig/AntennaTracker_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs/AP_BoardConfig/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_BoardConfig/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_BoardConfig/AntennaTracker ${objs/AP_BoardConfig/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_BoardConfig/AntennaTracker mavlink)
target_link_libraries(objs/AP_BoardConfig/AntennaTracker dronecan)
target_link_libraries(objs/AP_BoardConfig/AntennaTracker littlefs)

set(objs/AP_HAL/AntennaTracker_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs/AP_HAL/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_HAL/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_HAL/AntennaTracker ${objs/AP_HAL/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_HAL/AntennaTracker mavlink)
target_link_libraries(objs/AP_HAL/AntennaTracker dronecan)
target_link_libraries(objs/AP_HAL/AntennaTracker littlefs)

set(objs/AP_InertialSensor/AntennaTracker_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs/AP_InertialSensor/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_InertialSensor/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_InertialSensor/AntennaTracker ${objs/AP_InertialSensor/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_InertialSensor/AntennaTracker mavlink)
target_link_libraries(objs/AP_InertialSensor/AntennaTracker dronecan)
target_link_libraries(objs/AP_InertialSensor/AntennaTracker littlefs)

set(objs/AP_Math/AntennaTracker_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs/AP_Math/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Math/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Math/AntennaTracker ${objs/AP_Math/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Math/AntennaTracker mavlink)
target_link_libraries(objs/AP_Math/AntennaTracker dronecan)
target_link_libraries(objs/AP_Math/AntennaTracker littlefs)

set(objs/AP_Mission/AntennaTracker_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs/AP_Mission/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Mission/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Mission/AntennaTracker ${objs/AP_Mission/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Mission/AntennaTracker mavlink)
target_link_libraries(objs/AP_Mission/AntennaTracker dronecan)
target_link_libraries(objs/AP_Mission/AntennaTracker littlefs)

set(objs/AP_DAL/AntennaTracker_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs/AP_DAL/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_DAL/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_DAL/AntennaTracker ${objs/AP_DAL/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_DAL/AntennaTracker mavlink)
target_link_libraries(objs/AP_DAL/AntennaTracker dronecan)
target_link_libraries(objs/AP_DAL/AntennaTracker littlefs)

set(objs/AP_NavEKF2/AntennaTracker_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs/AP_NavEKF2/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_NavEKF2/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_NavEKF2/AntennaTracker ${objs/AP_NavEKF2/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_NavEKF2/AntennaTracker mavlink)
target_link_libraries(objs/AP_NavEKF2/AntennaTracker dronecan)
target_link_libraries(objs/AP_NavEKF2/AntennaTracker littlefs)

set(objs/AP_NavEKF3/AntennaTracker_SOURCES
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

set(objs/AP_NavEKF3/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_NavEKF3/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_NavEKF3/AntennaTracker ${objs/AP_NavEKF3/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_NavEKF3/AntennaTracker mavlink)
target_link_libraries(objs/AP_NavEKF3/AntennaTracker dronecan)
target_link_libraries(objs/AP_NavEKF3/AntennaTracker littlefs)

set(objs/AP_Rally/AntennaTracker_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs/AP_Rally/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Rally/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Rally/AntennaTracker ${objs/AP_Rally/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Rally/AntennaTracker mavlink)
target_link_libraries(objs/AP_Rally/AntennaTracker dronecan)
target_link_libraries(objs/AP_Rally/AntennaTracker littlefs)

set(objs/AP_Scheduler/AntennaTracker_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs/AP_Scheduler/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Scheduler/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Scheduler/AntennaTracker ${objs/AP_Scheduler/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Scheduler/AntennaTracker mavlink)
target_link_libraries(objs/AP_Scheduler/AntennaTracker dronecan)
target_link_libraries(objs/AP_Scheduler/AntennaTracker littlefs)

set(objs/AP_Terrain/AntennaTracker_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs/AP_Terrain/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Terrain/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Terrain/AntennaTracker ${objs/AP_Terrain/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Terrain/AntennaTracker mavlink)
target_link_libraries(objs/AP_Terrain/AntennaTracker dronecan)
target_link_libraries(objs/AP_Terrain/AntennaTracker littlefs)

set(objs/AP_Vehicle/AntennaTracker_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs/AP_Vehicle/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Vehicle/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Vehicle/AntennaTracker ${objs/AP_Vehicle/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Vehicle/AntennaTracker mavlink)
target_link_libraries(objs/AP_Vehicle/AntennaTracker dronecan)
target_link_libraries(objs/AP_Vehicle/AntennaTracker littlefs)

set(objs/AP_Logger/AntennaTracker_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs/AP_Logger/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Logger/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Logger/AntennaTracker ${objs/AP_Logger/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Logger/AntennaTracker mavlink)
target_link_libraries(objs/AP_Logger/AntennaTracker dronecan)
target_link_libraries(objs/AP_Logger/AntennaTracker littlefs)

set(objs/Filter/AntennaTracker_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs/Filter/AntennaTracker_INCLUDES
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

include_directories(${objs/Filter/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/Filter/AntennaTracker ${objs/Filter/AntennaTracker_SOURCES})

target_link_libraries(objs/Filter/AntennaTracker mavlink)
target_link_libraries(objs/Filter/AntennaTracker dronecan)
target_link_libraries(objs/Filter/AntennaTracker littlefs)

set(objs/GCS_MAVLink/AntennaTracker_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs/GCS_MAVLink/AntennaTracker_INCLUDES
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

include_directories(${objs/GCS_MAVLink/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/GCS_MAVLink/AntennaTracker ${objs/GCS_MAVLink/AntennaTracker_SOURCES})

target_link_libraries(objs/GCS_MAVLink/AntennaTracker mavlink)
target_link_libraries(objs/GCS_MAVLink/AntennaTracker dronecan)
target_link_libraries(objs/GCS_MAVLink/AntennaTracker littlefs)

set(objs/RC_Channel/AntennaTracker_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs/RC_Channel/AntennaTracker_INCLUDES
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

include_directories(${objs/RC_Channel/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/RC_Channel/AntennaTracker ${objs/RC_Channel/AntennaTracker_SOURCES})

target_link_libraries(objs/RC_Channel/AntennaTracker mavlink)
target_link_libraries(objs/RC_Channel/AntennaTracker dronecan)
target_link_libraries(objs/RC_Channel/AntennaTracker littlefs)

set(objs/StorageManager/AntennaTracker_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs/StorageManager/AntennaTracker_INCLUDES
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

include_directories(${objs/StorageManager/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/StorageManager/AntennaTracker ${objs/StorageManager/AntennaTracker_SOURCES})

target_link_libraries(objs/StorageManager/AntennaTracker mavlink)
target_link_libraries(objs/StorageManager/AntennaTracker dronecan)
target_link_libraries(objs/StorageManager/AntennaTracker littlefs)

set(objs/AP_Frsky_Telem/AntennaTracker_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs/AP_Frsky_Telem/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Frsky_Telem/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Frsky_Telem/AntennaTracker ${objs/AP_Frsky_Telem/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Frsky_Telem/AntennaTracker mavlink)
target_link_libraries(objs/AP_Frsky_Telem/AntennaTracker dronecan)
target_link_libraries(objs/AP_Frsky_Telem/AntennaTracker littlefs)

set(objs/AP_IBus_Telem/AntennaTracker_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs/AP_IBus_Telem/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_IBus_Telem/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_IBus_Telem/AntennaTracker ${objs/AP_IBus_Telem/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_IBus_Telem/AntennaTracker mavlink)
target_link_libraries(objs/AP_IBus_Telem/AntennaTracker dronecan)
target_link_libraries(objs/AP_IBus_Telem/AntennaTracker littlefs)

set(objs/AP_Relay/AntennaTracker_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs/AP_Relay/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Relay/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Relay/AntennaTracker ${objs/AP_Relay/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Relay/AntennaTracker mavlink)
target_link_libraries(objs/AP_Relay/AntennaTracker dronecan)
target_link_libraries(objs/AP_Relay/AntennaTracker littlefs)

set(objs/AP_RCProtocol/AntennaTracker_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs/AP_RCProtocol/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_RCProtocol/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_RCProtocol/AntennaTracker ${objs/AP_RCProtocol/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_RCProtocol/AntennaTracker mavlink)
target_link_libraries(objs/AP_RCProtocol/AntennaTracker dronecan)
target_link_libraries(objs/AP_RCProtocol/AntennaTracker littlefs)

set(objs/AP_BLHeli/AntennaTracker_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs/AP_BLHeli/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_BLHeli/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_BLHeli/AntennaTracker ${objs/AP_BLHeli/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_BLHeli/AntennaTracker mavlink)
target_link_libraries(objs/AP_BLHeli/AntennaTracker dronecan)
target_link_libraries(objs/AP_BLHeli/AntennaTracker littlefs)

set(objs/AP_Proximity/AntennaTracker_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs/AP_Proximity/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Proximity/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Proximity/AntennaTracker ${objs/AP_Proximity/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Proximity/AntennaTracker mavlink)
target_link_libraries(objs/AP_Proximity/AntennaTracker dronecan)
target_link_libraries(objs/AP_Proximity/AntennaTracker littlefs)

set(objs/AC_Fence/AntennaTracker_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs/AC_Fence/AntennaTracker_INCLUDES
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

include_directories(${objs/AC_Fence/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AC_Fence/AntennaTracker ${objs/AC_Fence/AntennaTracker_SOURCES})

target_link_libraries(objs/AC_Fence/AntennaTracker mavlink)
target_link_libraries(objs/AC_Fence/AntennaTracker dronecan)
target_link_libraries(objs/AC_Fence/AntennaTracker littlefs)

set(objs/AC_Avoidance/AntennaTracker_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs/AC_Avoidance/AntennaTracker_INCLUDES
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

include_directories(${objs/AC_Avoidance/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AC_Avoidance/AntennaTracker ${objs/AC_Avoidance/AntennaTracker_SOURCES})

target_link_libraries(objs/AC_Avoidance/AntennaTracker mavlink)
target_link_libraries(objs/AC_Avoidance/AntennaTracker dronecan)
target_link_libraries(objs/AC_Avoidance/AntennaTracker littlefs)

set(objs/AP_LandingGear/AntennaTracker_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs/AP_LandingGear/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_LandingGear/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_LandingGear/AntennaTracker ${objs/AP_LandingGear/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_LandingGear/AntennaTracker mavlink)
target_link_libraries(objs/AP_LandingGear/AntennaTracker dronecan)
target_link_libraries(objs/AP_LandingGear/AntennaTracker littlefs)

set(objs/AP_OSD/AntennaTracker_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs/AP_OSD/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_OSD/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_OSD/AntennaTracker ${objs/AP_OSD/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_OSD/AntennaTracker mavlink)
target_link_libraries(objs/AP_OSD/AntennaTracker dronecan)
target_link_libraries(objs/AP_OSD/AntennaTracker littlefs)

set(objs/AP_Filesystem/AntennaTracker_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs/AP_Filesystem/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Filesystem/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Filesystem/AntennaTracker ${objs/AP_Filesystem/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Filesystem/AntennaTracker mavlink)
target_link_libraries(objs/AP_Filesystem/AntennaTracker dronecan)
target_link_libraries(objs/AP_Filesystem/AntennaTracker littlefs)

set(objs/AP_ADSB/AntennaTracker_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs/AP_ADSB/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_ADSB/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_ADSB/AntennaTracker ${objs/AP_ADSB/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_ADSB/AntennaTracker mavlink)
target_link_libraries(objs/AP_ADSB/AntennaTracker dronecan)
target_link_libraries(objs/AP_ADSB/AntennaTracker littlefs)

set(objs/AP_ADSB/sagetech-sdk_SOURCES
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

set(objs/AP_ADSB/sagetech-sdk_INCLUDES
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

include_directories(${objs/AP_ADSB/sagetech-sdk_INCLUDES})

add_library(objs/AP_ADSB/sagetech-sdk ${objs/AP_ADSB/sagetech-sdk_SOURCES})

target_link_libraries(objs/AP_ADSB/sagetech-sdk mavlink)
target_link_libraries(objs/AP_ADSB/sagetech-sdk dronecan)
target_link_libraries(objs/AP_ADSB/sagetech-sdk littlefs)

set(objs/AP_ESC_Telem/AntennaTracker_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs/AP_ESC_Telem/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_ESC_Telem/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_ESC_Telem/AntennaTracker ${objs/AP_ESC_Telem/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_ESC_Telem/AntennaTracker mavlink)
target_link_libraries(objs/AP_ESC_Telem/AntennaTracker dronecan)
target_link_libraries(objs/AP_ESC_Telem/AntennaTracker littlefs)

set(objs/AP_GyroFFT/AntennaTracker_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs/AP_GyroFFT/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_GyroFFT/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_GyroFFT/AntennaTracker ${objs/AP_GyroFFT/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_GyroFFT/AntennaTracker mavlink)
target_link_libraries(objs/AP_GyroFFT/AntennaTracker dronecan)
target_link_libraries(objs/AP_GyroFFT/AntennaTracker littlefs)

set(objs/AP_RCTelemetry/AntennaTracker_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs/AP_RCTelemetry/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_RCTelemetry/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_RCTelemetry/AntennaTracker ${objs/AP_RCTelemetry/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_RCTelemetry/AntennaTracker mavlink)
target_link_libraries(objs/AP_RCTelemetry/AntennaTracker dronecan)
target_link_libraries(objs/AP_RCTelemetry/AntennaTracker littlefs)

set(objs/AP_ExternalAHRS/AntennaTracker_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs/AP_ExternalAHRS/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_ExternalAHRS/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_ExternalAHRS/AntennaTracker ${objs/AP_ExternalAHRS/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_ExternalAHRS/AntennaTracker mavlink)
target_link_libraries(objs/AP_ExternalAHRS/AntennaTracker dronecan)
target_link_libraries(objs/AP_ExternalAHRS/AntennaTracker littlefs)

set(objs/AP_TemperatureSensor/AntennaTracker_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs/AP_TemperatureSensor/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_TemperatureSensor/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_TemperatureSensor/AntennaTracker ${objs/AP_TemperatureSensor/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_TemperatureSensor/AntennaTracker mavlink)
target_link_libraries(objs/AP_TemperatureSensor/AntennaTracker dronecan)
target_link_libraries(objs/AP_TemperatureSensor/AntennaTracker littlefs)

set(objs/AP_AIS/AntennaTracker_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs/AP_AIS/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_AIS/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_AIS/AntennaTracker ${objs/AP_AIS/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_AIS/AntennaTracker mavlink)
target_link_libraries(objs/AP_AIS/AntennaTracker dronecan)
target_link_libraries(objs/AP_AIS/AntennaTracker littlefs)

set(objs/AP_Arming/AntennaTracker_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs/AP_Arming/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Arming/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Arming/AntennaTracker ${objs/AP_Arming/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Arming/AntennaTracker mavlink)
target_link_libraries(objs/AP_Arming/AntennaTracker dronecan)
target_link_libraries(objs/AP_Arming/AntennaTracker littlefs)

set(objs/AP_Follow/AntennaTracker_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs/AP_Follow/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Follow/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Follow/AntennaTracker ${objs/AP_Follow/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Follow/AntennaTracker mavlink)
target_link_libraries(objs/AP_Follow/AntennaTracker dronecan)
target_link_libraries(objs/AP_Follow/AntennaTracker littlefs)

set(objs/AP_HAL_SITL/AntennaTracker_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs/AP_HAL_SITL/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_HAL_SITL/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_HAL_SITL/AntennaTracker ${objs/AP_HAL_SITL/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_HAL_SITL/AntennaTracker mavlink)
target_link_libraries(objs/AP_HAL_SITL/AntennaTracker dronecan)
target_link_libraries(objs/AP_HAL_SITL/AntennaTracker littlefs)

set(objs/SITL/AntennaTracker_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs/SITL/AntennaTracker_INCLUDES
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

include_directories(${objs/SITL/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/SITL/AntennaTracker ${objs/SITL/AntennaTracker_SOURCES})

target_link_libraries(objs/SITL/AntennaTracker mavlink)
target_link_libraries(objs/SITL/AntennaTracker dronecan)
target_link_libraries(objs/SITL/AntennaTracker littlefs)

set(objs/AP_Scripting/AntennaTracker_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs/AP_Scripting/AntennaTracker_INCLUDES
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

include_directories(${objs/AP_Scripting/AntennaTracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_library(objs/AP_Scripting/AntennaTracker ${objs/AP_Scripting/AntennaTracker_SOURCES})

target_link_libraries(objs/AP_Scripting/AntennaTracker mavlink)
target_link_libraries(objs/AP_Scripting/AntennaTracker dronecan)
target_link_libraries(objs/AP_Scripting/AntennaTracker littlefs)

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

target_link_libraries(AntennaTracker_libs objs/AP_AccelCal)
target_link_libraries(AntennaTracker_libs objs/AP_AccelCal/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ADC)
target_link_libraries(AntennaTracker_libs objs/AP_ADC/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_AHRS)
target_link_libraries(AntennaTracker_libs objs/AP_AHRS/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Airspeed)
target_link_libraries(AntennaTracker_libs objs/AP_Airspeed/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Baro)
target_link_libraries(AntennaTracker_libs objs/AP_Baro/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_BattMonitor)
target_link_libraries(AntennaTracker_libs objs/AP_BattMonitor/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_BoardConfig)
target_link_libraries(AntennaTracker_libs objs/AP_BoardConfig/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Camera)
target_link_libraries(AntennaTracker_libs objs/AP_Camera/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Common)
target_link_libraries(AntennaTracker_libs objs/AP_Common/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Compass)
target_link_libraries(AntennaTracker_libs objs/AP_Compass/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Declination)
target_link_libraries(AntennaTracker_libs objs/AP_Declination/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_GPS)
target_link_libraries(AntennaTracker_libs objs/AP_GPS/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_GSOF)
target_link_libraries(AntennaTracker_libs objs/AP_GSOF/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_HAL)
target_link_libraries(AntennaTracker_libs objs/AP_HAL/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_HAL_Empty)
target_link_libraries(AntennaTracker_libs objs/AP_HAL_Empty/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_DDS)
target_link_libraries(AntennaTracker_libs objs/AP_DDS/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_InertialSensor)
target_link_libraries(AntennaTracker_libs objs/AP_InertialSensor/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Math)
target_link_libraries(AntennaTracker_libs objs/AP_Math/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Mission)
target_link_libraries(AntennaTracker_libs objs/AP_Mission/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_DAL)
target_link_libraries(AntennaTracker_libs objs/AP_DAL/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_NavEKF)
target_link_libraries(AntennaTracker_libs objs/AP_NavEKF/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_NavEKF2)
target_link_libraries(AntennaTracker_libs objs/AP_NavEKF2/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_NavEKF3)
target_link_libraries(AntennaTracker_libs objs/AP_NavEKF3/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Notify)
target_link_libraries(AntennaTracker_libs objs/AP_Notify/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_OpticalFlow)
target_link_libraries(AntennaTracker_libs objs/AP_OpticalFlow/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Param)
target_link_libraries(AntennaTracker_libs objs/AP_Param/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Rally)
target_link_libraries(AntennaTracker_libs objs/AP_Rally/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_LightWareSerial)
target_link_libraries(AntennaTracker_libs objs/AP_LightWareSerial/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RangeFinder)
target_link_libraries(AntennaTracker_libs objs/AP_RangeFinder/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Scheduler)
target_link_libraries(AntennaTracker_libs objs/AP_Scheduler/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_SerialManager)
target_link_libraries(AntennaTracker_libs objs/AP_SerialManager/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Terrain)
target_link_libraries(AntennaTracker_libs objs/AP_Terrain/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Vehicle)
target_link_libraries(AntennaTracker_libs objs/AP_Vehicle/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_InternalError)
target_link_libraries(AntennaTracker_libs objs/AP_InternalError/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Logger)
target_link_libraries(AntennaTracker_libs objs/AP_Logger/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/Filter)
target_link_libraries(AntennaTracker_libs objs/Filter/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/GCS_MAVLink)
target_link_libraries(AntennaTracker_libs objs/GCS_MAVLink/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/RC_Channel)
target_link_libraries(AntennaTracker_libs objs/RC_Channel/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/SRV_Channel)
target_link_libraries(AntennaTracker_libs objs/SRV_Channel/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/StorageManager)
target_link_libraries(AntennaTracker_libs objs/StorageManager/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Tuning)
target_link_libraries(AntennaTracker_libs objs/AP_Tuning/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RPM)
target_link_libraries(AntennaTracker_libs objs/AP_RPM/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RSSI)
target_link_libraries(AntennaTracker_libs objs/AP_RSSI/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Mount)
target_link_libraries(AntennaTracker_libs objs/AP_Mount/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Module)
target_link_libraries(AntennaTracker_libs objs/AP_Module/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Button)
target_link_libraries(AntennaTracker_libs objs/AP_Button/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ICEngine)
target_link_libraries(AntennaTracker_libs objs/AP_ICEngine/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Networking)
target_link_libraries(AntennaTracker_libs objs/AP_Networking/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Frsky_Telem)
target_link_libraries(AntennaTracker_libs objs/AP_Frsky_Telem/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_IBus_Telem)
target_link_libraries(AntennaTracker_libs objs/AP_IBus_Telem/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_FlashStorage)
target_link_libraries(AntennaTracker_libs objs/AP_FlashStorage/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Relay)
target_link_libraries(AntennaTracker_libs objs/AP_Relay/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ServoRelayEvents)
target_link_libraries(AntennaTracker_libs objs/AP_ServoRelayEvents/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Volz_Protocol)
target_link_libraries(AntennaTracker_libs objs/AP_Volz_Protocol/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_SBusOut)
target_link_libraries(AntennaTracker_libs objs/AP_SBusOut/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_IOMCU)
target_link_libraries(AntennaTracker_libs objs/AP_IOMCU/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Parachute)
target_link_libraries(AntennaTracker_libs objs/AP_Parachute/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RAMTRON)
target_link_libraries(AntennaTracker_libs objs/AP_RAMTRON/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RCProtocol)
target_link_libraries(AntennaTracker_libs objs/AP_RCProtocol/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Radio)
target_link_libraries(AntennaTracker_libs objs/AP_Radio/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_TempCalibration)
target_link_libraries(AntennaTracker_libs objs/AP_TempCalibration/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_VisualOdom)
target_link_libraries(AntennaTracker_libs objs/AP_VisualOdom/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_BLHeli)
target_link_libraries(AntennaTracker_libs objs/AP_BLHeli/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ROMFS)
target_link_libraries(AntennaTracker_libs objs/AP_ROMFS/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Proximity)
target_link_libraries(AntennaTracker_libs objs/AP_Proximity/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Gripper)
target_link_libraries(AntennaTracker_libs objs/AP_Gripper/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RTC)
target_link_libraries(AntennaTracker_libs objs/AP_RTC/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AC_Sprayer)
target_link_libraries(AntennaTracker_libs objs/AC_Sprayer/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AC_Fence)
target_link_libraries(AntennaTracker_libs objs/AC_Fence/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AC_Avoidance)
target_link_libraries(AntennaTracker_libs objs/AC_Avoidance/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_LandingGear)
target_link_libraries(AntennaTracker_libs objs/AP_LandingGear/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RobotisServo)
target_link_libraries(AntennaTracker_libs objs/AP_RobotisServo/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_NMEA_Output)
target_link_libraries(AntennaTracker_libs objs/AP_NMEA_Output/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_OSD)
target_link_libraries(AntennaTracker_libs objs/AP_OSD/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Filesystem)
target_link_libraries(AntennaTracker_libs objs/AP_Filesystem/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ADSB)
target_link_libraries(AntennaTracker_libs objs/AP_ADSB/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ADSB/sagetech-sdk)
target_link_libraries(AntennaTracker_libs objs/AP_ADSB/sagetech-sdk/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AC_PID)
target_link_libraries(AntennaTracker_libs objs/AC_PID/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_SerialLED)
target_link_libraries(AntennaTracker_libs objs/AP_SerialLED/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_EFI)
target_link_libraries(AntennaTracker_libs objs/AP_EFI/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Hott_Telem)
target_link_libraries(AntennaTracker_libs objs/AP_Hott_Telem/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ESC_Telem)
target_link_libraries(AntennaTracker_libs objs/AP_ESC_Telem/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Servo_Telem)
target_link_libraries(AntennaTracker_libs objs/AP_Servo_Telem/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Stats)
target_link_libraries(AntennaTracker_libs objs/AP_Stats/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_GyroFFT)
target_link_libraries(AntennaTracker_libs objs/AP_GyroFFT/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RCTelemetry)
target_link_libraries(AntennaTracker_libs objs/AP_RCTelemetry/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Generator)
target_link_libraries(AntennaTracker_libs objs/AP_Generator/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_MSP)
target_link_libraries(AntennaTracker_libs objs/AP_MSP/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_OLC)
target_link_libraries(AntennaTracker_libs objs/AP_OLC/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_WheelEncoder)
target_link_libraries(AntennaTracker_libs objs/AP_WheelEncoder/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ExternalAHRS)
target_link_libraries(AntennaTracker_libs objs/AP_ExternalAHRS/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_VideoTX)
target_link_libraries(AntennaTracker_libs objs/AP_VideoTX/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_FETtecOneWire)
target_link_libraries(AntennaTracker_libs objs/AP_FETtecOneWire/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_TemperatureSensor)
target_link_libraries(AntennaTracker_libs objs/AP_TemperatureSensor/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Torqeedo)
target_link_libraries(AntennaTracker_libs objs/AP_Torqeedo/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_CustomRotations)
target_link_libraries(AntennaTracker_libs objs/AP_CustomRotations/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_AIS)
target_link_libraries(AntennaTracker_libs objs/AP_AIS/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_OpenDroneID)
target_link_libraries(AntennaTracker_libs objs/AP_OpenDroneID/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_CheckFirmware)
target_link_libraries(AntennaTracker_libs objs/AP_CheckFirmware/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_ExternalControl)
target_link_libraries(AntennaTracker_libs objs/AP_ExternalControl/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_JSON)
target_link_libraries(AntennaTracker_libs objs/AP_JSON/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Beacon)
target_link_libraries(AntennaTracker_libs objs/AP_Beacon/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Arming)
target_link_libraries(AntennaTracker_libs objs/AP_Arming/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_RCMapper)
target_link_libraries(AntennaTracker_libs objs/AP_RCMapper/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_MultiHeap)
target_link_libraries(AntennaTracker_libs objs/AP_MultiHeap/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Follow)
target_link_libraries(AntennaTracker_libs objs/AP_Follow/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_CANManager)
target_link_libraries(AntennaTracker_libs objs/AP_CANManager/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_KDECAN)
target_link_libraries(AntennaTracker_libs objs/AP_KDECAN/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_PiccoloCAN)
target_link_libraries(AntennaTracker_libs objs/AP_PiccoloCAN/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_PiccoloCAN/piccolo_protocol)
target_link_libraries(AntennaTracker_libs objs/AP_PiccoloCAN/piccolo_protocol/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_DroneCAN)
target_link_libraries(AntennaTracker_libs objs/AP_DroneCAN/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/modules/DroneCAN/libcanard/*.c)
target_link_libraries(AntennaTracker_libs objs/modules/DroneCAN/libcanard/*.c/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_HAL_SITL)
target_link_libraries(AntennaTracker_libs objs/AP_HAL_SITL/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_CSVReader)
target_link_libraries(AntennaTracker_libs objs/AP_CSVReader/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/SITL)
target_link_libraries(AntennaTracker_libs objs/SITL/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Scripting)
target_link_libraries(AntennaTracker_libs objs/AP_Scripting/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AP_Scripting/lua/src)
target_link_libraries(AntennaTracker_libs objs/AP_Scripting/lua/src/AntennaTracker)
target_link_libraries(AntennaTracker_libs objs/AC_CustomControl)
target_link_libraries(AntennaTracker_libs objs/AC_CustomControl/AntennaTracker)

set(bin/antennatracker_SOURCES
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

set(bin/antennatracker_INCLUDES
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

include_directories(${bin/antennatracker_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker")

add_executable(bin/antennatracker ${bin/antennatracker_SOURCES})

target_link_libraries(bin/antennatracker A)
target_link_libraries(bin/antennatracker n)
target_link_libraries(bin/antennatracker t)
target_link_libraries(bin/antennatracker e)
target_link_libraries(bin/antennatracker n)
target_link_libraries(bin/antennatracker n)
target_link_libraries(bin/antennatracker a)
target_link_libraries(bin/antennatracker T)
target_link_libraries(bin/antennatracker r)
target_link_libraries(bin/antennatracker a)
target_link_libraries(bin/antennatracker c)
target_link_libraries(bin/antennatracker k)
target_link_libraries(bin/antennatracker e)
target_link_libraries(bin/antennatracker r)
target_link_libraries(bin/antennatracker _)
target_link_libraries(bin/antennatracker l)
target_link_libraries(bin/antennatracker i)
target_link_libraries(bin/antennatracker b)
target_link_libraries(bin/antennatracker s)

