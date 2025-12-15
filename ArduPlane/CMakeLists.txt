
set(objs/AP_AHRS/ArduPlane_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs/AP_AHRS/ArduPlane_INCLUDES
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

include_directories(${objs/AP_AHRS/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_AHRS/ArduPlane ${objs/AP_AHRS/ArduPlane_SOURCES})

target_link_libraries(objs/AP_AHRS/ArduPlane mavlink)
target_link_libraries(objs/AP_AHRS/ArduPlane dronecan)
target_link_libraries(objs/AP_AHRS/ArduPlane littlefs)

set(objs/AP_Airspeed/ArduPlane_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs/AP_Airspeed/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Airspeed/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Airspeed/ArduPlane ${objs/AP_Airspeed/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Airspeed/ArduPlane mavlink)
target_link_libraries(objs/AP_Airspeed/ArduPlane dronecan)
target_link_libraries(objs/AP_Airspeed/ArduPlane littlefs)

set(objs/AP_Baro/ArduPlane_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs/AP_Baro/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Baro/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Baro/ArduPlane ${objs/AP_Baro/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Baro/ArduPlane mavlink)
target_link_libraries(objs/AP_Baro/ArduPlane dronecan)
target_link_libraries(objs/AP_Baro/ArduPlane littlefs)

set(objs/AP_BattMonitor/ArduPlane_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs/AP_BattMonitor/ArduPlane_INCLUDES
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

include_directories(${objs/AP_BattMonitor/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_BattMonitor/ArduPlane ${objs/AP_BattMonitor/ArduPlane_SOURCES})

target_link_libraries(objs/AP_BattMonitor/ArduPlane mavlink)
target_link_libraries(objs/AP_BattMonitor/ArduPlane dronecan)
target_link_libraries(objs/AP_BattMonitor/ArduPlane littlefs)

set(objs/AP_BoardConfig/ArduPlane_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs/AP_BoardConfig/ArduPlane_INCLUDES
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

include_directories(${objs/AP_BoardConfig/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_BoardConfig/ArduPlane ${objs/AP_BoardConfig/ArduPlane_SOURCES})

target_link_libraries(objs/AP_BoardConfig/ArduPlane mavlink)
target_link_libraries(objs/AP_BoardConfig/ArduPlane dronecan)
target_link_libraries(objs/AP_BoardConfig/ArduPlane littlefs)

set(objs/AP_HAL/ArduPlane_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs/AP_HAL/ArduPlane_INCLUDES
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

include_directories(${objs/AP_HAL/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_HAL/ArduPlane ${objs/AP_HAL/ArduPlane_SOURCES})

target_link_libraries(objs/AP_HAL/ArduPlane mavlink)
target_link_libraries(objs/AP_HAL/ArduPlane dronecan)
target_link_libraries(objs/AP_HAL/ArduPlane littlefs)

set(objs/AP_InertialSensor/ArduPlane_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs/AP_InertialSensor/ArduPlane_INCLUDES
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

include_directories(${objs/AP_InertialSensor/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_InertialSensor/ArduPlane ${objs/AP_InertialSensor/ArduPlane_SOURCES})

target_link_libraries(objs/AP_InertialSensor/ArduPlane mavlink)
target_link_libraries(objs/AP_InertialSensor/ArduPlane dronecan)
target_link_libraries(objs/AP_InertialSensor/ArduPlane littlefs)

set(objs/AP_Math/ArduPlane_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs/AP_Math/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Math/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Math/ArduPlane ${objs/AP_Math/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Math/ArduPlane mavlink)
target_link_libraries(objs/AP_Math/ArduPlane dronecan)
target_link_libraries(objs/AP_Math/ArduPlane littlefs)

set(objs/AP_Mission/ArduPlane_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs/AP_Mission/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Mission/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Mission/ArduPlane ${objs/AP_Mission/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Mission/ArduPlane mavlink)
target_link_libraries(objs/AP_Mission/ArduPlane dronecan)
target_link_libraries(objs/AP_Mission/ArduPlane littlefs)

set(objs/AP_DAL/ArduPlane_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs/AP_DAL/ArduPlane_INCLUDES
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

include_directories(${objs/AP_DAL/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_DAL/ArduPlane ${objs/AP_DAL/ArduPlane_SOURCES})

target_link_libraries(objs/AP_DAL/ArduPlane mavlink)
target_link_libraries(objs/AP_DAL/ArduPlane dronecan)
target_link_libraries(objs/AP_DAL/ArduPlane littlefs)

set(objs/AP_NavEKF2/ArduPlane_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs/AP_NavEKF2/ArduPlane_INCLUDES
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

include_directories(${objs/AP_NavEKF2/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_NavEKF2/ArduPlane ${objs/AP_NavEKF2/ArduPlane_SOURCES})

target_link_libraries(objs/AP_NavEKF2/ArduPlane mavlink)
target_link_libraries(objs/AP_NavEKF2/ArduPlane dronecan)
target_link_libraries(objs/AP_NavEKF2/ArduPlane littlefs)

set(objs/AP_NavEKF3/ArduPlane_SOURCES
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

set(objs/AP_NavEKF3/ArduPlane_INCLUDES
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

include_directories(${objs/AP_NavEKF3/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_NavEKF3/ArduPlane ${objs/AP_NavEKF3/ArduPlane_SOURCES})

target_link_libraries(objs/AP_NavEKF3/ArduPlane mavlink)
target_link_libraries(objs/AP_NavEKF3/ArduPlane dronecan)
target_link_libraries(objs/AP_NavEKF3/ArduPlane littlefs)

set(objs/AP_Rally/ArduPlane_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs/AP_Rally/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Rally/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Rally/ArduPlane ${objs/AP_Rally/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Rally/ArduPlane mavlink)
target_link_libraries(objs/AP_Rally/ArduPlane dronecan)
target_link_libraries(objs/AP_Rally/ArduPlane littlefs)

set(objs/AP_Scheduler/ArduPlane_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs/AP_Scheduler/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Scheduler/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Scheduler/ArduPlane ${objs/AP_Scheduler/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Scheduler/ArduPlane mavlink)
target_link_libraries(objs/AP_Scheduler/ArduPlane dronecan)
target_link_libraries(objs/AP_Scheduler/ArduPlane littlefs)

set(objs/AP_Terrain/ArduPlane_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs/AP_Terrain/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Terrain/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Terrain/ArduPlane ${objs/AP_Terrain/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Terrain/ArduPlane mavlink)
target_link_libraries(objs/AP_Terrain/ArduPlane dronecan)
target_link_libraries(objs/AP_Terrain/ArduPlane littlefs)

set(objs/AP_Vehicle/ArduPlane_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs/AP_Vehicle/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Vehicle/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Vehicle/ArduPlane ${objs/AP_Vehicle/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Vehicle/ArduPlane mavlink)
target_link_libraries(objs/AP_Vehicle/ArduPlane dronecan)
target_link_libraries(objs/AP_Vehicle/ArduPlane littlefs)

set(objs/AP_Logger/ArduPlane_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs/AP_Logger/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Logger/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Logger/ArduPlane ${objs/AP_Logger/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Logger/ArduPlane mavlink)
target_link_libraries(objs/AP_Logger/ArduPlane dronecan)
target_link_libraries(objs/AP_Logger/ArduPlane littlefs)

set(objs/Filter/ArduPlane_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs/Filter/ArduPlane_INCLUDES
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

include_directories(${objs/Filter/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/Filter/ArduPlane ${objs/Filter/ArduPlane_SOURCES})

target_link_libraries(objs/Filter/ArduPlane mavlink)
target_link_libraries(objs/Filter/ArduPlane dronecan)
target_link_libraries(objs/Filter/ArduPlane littlefs)

set(objs/GCS_MAVLink/ArduPlane_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs/GCS_MAVLink/ArduPlane_INCLUDES
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

include_directories(${objs/GCS_MAVLink/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/GCS_MAVLink/ArduPlane ${objs/GCS_MAVLink/ArduPlane_SOURCES})

target_link_libraries(objs/GCS_MAVLink/ArduPlane mavlink)
target_link_libraries(objs/GCS_MAVLink/ArduPlane dronecan)
target_link_libraries(objs/GCS_MAVLink/ArduPlane littlefs)

set(objs/RC_Channel/ArduPlane_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs/RC_Channel/ArduPlane_INCLUDES
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

include_directories(${objs/RC_Channel/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/RC_Channel/ArduPlane ${objs/RC_Channel/ArduPlane_SOURCES})

target_link_libraries(objs/RC_Channel/ArduPlane mavlink)
target_link_libraries(objs/RC_Channel/ArduPlane dronecan)
target_link_libraries(objs/RC_Channel/ArduPlane littlefs)

set(objs/StorageManager/ArduPlane_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs/StorageManager/ArduPlane_INCLUDES
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

include_directories(${objs/StorageManager/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/StorageManager/ArduPlane ${objs/StorageManager/ArduPlane_SOURCES})

target_link_libraries(objs/StorageManager/ArduPlane mavlink)
target_link_libraries(objs/StorageManager/ArduPlane dronecan)
target_link_libraries(objs/StorageManager/ArduPlane littlefs)

set(objs/AP_Frsky_Telem/ArduPlane_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs/AP_Frsky_Telem/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Frsky_Telem/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Frsky_Telem/ArduPlane ${objs/AP_Frsky_Telem/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Frsky_Telem/ArduPlane mavlink)
target_link_libraries(objs/AP_Frsky_Telem/ArduPlane dronecan)
target_link_libraries(objs/AP_Frsky_Telem/ArduPlane littlefs)

set(objs/AP_IBus_Telem/ArduPlane_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs/AP_IBus_Telem/ArduPlane_INCLUDES
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

include_directories(${objs/AP_IBus_Telem/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_IBus_Telem/ArduPlane ${objs/AP_IBus_Telem/ArduPlane_SOURCES})

target_link_libraries(objs/AP_IBus_Telem/ArduPlane mavlink)
target_link_libraries(objs/AP_IBus_Telem/ArduPlane dronecan)
target_link_libraries(objs/AP_IBus_Telem/ArduPlane littlefs)

set(objs/AP_Relay/ArduPlane_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs/AP_Relay/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Relay/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Relay/ArduPlane ${objs/AP_Relay/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Relay/ArduPlane mavlink)
target_link_libraries(objs/AP_Relay/ArduPlane dronecan)
target_link_libraries(objs/AP_Relay/ArduPlane littlefs)

set(objs/AP_RCProtocol/ArduPlane_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs/AP_RCProtocol/ArduPlane_INCLUDES
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

include_directories(${objs/AP_RCProtocol/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_RCProtocol/ArduPlane ${objs/AP_RCProtocol/ArduPlane_SOURCES})

target_link_libraries(objs/AP_RCProtocol/ArduPlane mavlink)
target_link_libraries(objs/AP_RCProtocol/ArduPlane dronecan)
target_link_libraries(objs/AP_RCProtocol/ArduPlane littlefs)

set(objs/AP_BLHeli/ArduPlane_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs/AP_BLHeli/ArduPlane_INCLUDES
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

include_directories(${objs/AP_BLHeli/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_BLHeli/ArduPlane ${objs/AP_BLHeli/ArduPlane_SOURCES})

target_link_libraries(objs/AP_BLHeli/ArduPlane mavlink)
target_link_libraries(objs/AP_BLHeli/ArduPlane dronecan)
target_link_libraries(objs/AP_BLHeli/ArduPlane littlefs)

set(objs/AP_Proximity/ArduPlane_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs/AP_Proximity/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Proximity/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Proximity/ArduPlane ${objs/AP_Proximity/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Proximity/ArduPlane mavlink)
target_link_libraries(objs/AP_Proximity/ArduPlane dronecan)
target_link_libraries(objs/AP_Proximity/ArduPlane littlefs)

set(objs/AC_Fence/ArduPlane_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs/AC_Fence/ArduPlane_INCLUDES
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

include_directories(${objs/AC_Fence/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AC_Fence/ArduPlane ${objs/AC_Fence/ArduPlane_SOURCES})

target_link_libraries(objs/AC_Fence/ArduPlane mavlink)
target_link_libraries(objs/AC_Fence/ArduPlane dronecan)
target_link_libraries(objs/AC_Fence/ArduPlane littlefs)

set(objs/AC_Avoidance/ArduPlane_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs/AC_Avoidance/ArduPlane_INCLUDES
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

include_directories(${objs/AC_Avoidance/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AC_Avoidance/ArduPlane ${objs/AC_Avoidance/ArduPlane_SOURCES})

target_link_libraries(objs/AC_Avoidance/ArduPlane mavlink)
target_link_libraries(objs/AC_Avoidance/ArduPlane dronecan)
target_link_libraries(objs/AC_Avoidance/ArduPlane littlefs)

set(objs/AP_LandingGear/ArduPlane_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs/AP_LandingGear/ArduPlane_INCLUDES
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

include_directories(${objs/AP_LandingGear/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_LandingGear/ArduPlane ${objs/AP_LandingGear/ArduPlane_SOURCES})

target_link_libraries(objs/AP_LandingGear/ArduPlane mavlink)
target_link_libraries(objs/AP_LandingGear/ArduPlane dronecan)
target_link_libraries(objs/AP_LandingGear/ArduPlane littlefs)

set(objs/AP_OSD/ArduPlane_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs/AP_OSD/ArduPlane_INCLUDES
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

include_directories(${objs/AP_OSD/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_OSD/ArduPlane ${objs/AP_OSD/ArduPlane_SOURCES})

target_link_libraries(objs/AP_OSD/ArduPlane mavlink)
target_link_libraries(objs/AP_OSD/ArduPlane dronecan)
target_link_libraries(objs/AP_OSD/ArduPlane littlefs)

set(objs/AP_Filesystem/ArduPlane_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs/AP_Filesystem/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Filesystem/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Filesystem/ArduPlane ${objs/AP_Filesystem/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Filesystem/ArduPlane mavlink)
target_link_libraries(objs/AP_Filesystem/ArduPlane dronecan)
target_link_libraries(objs/AP_Filesystem/ArduPlane littlefs)

set(objs/AP_ADSB/ArduPlane_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs/AP_ADSB/ArduPlane_INCLUDES
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

include_directories(${objs/AP_ADSB/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_ADSB/ArduPlane ${objs/AP_ADSB/ArduPlane_SOURCES})

target_link_libraries(objs/AP_ADSB/ArduPlane mavlink)
target_link_libraries(objs/AP_ADSB/ArduPlane dronecan)
target_link_libraries(objs/AP_ADSB/ArduPlane littlefs)

set(objs/AP_ESC_Telem/ArduPlane_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs/AP_ESC_Telem/ArduPlane_INCLUDES
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

include_directories(${objs/AP_ESC_Telem/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_ESC_Telem/ArduPlane ${objs/AP_ESC_Telem/ArduPlane_SOURCES})

target_link_libraries(objs/AP_ESC_Telem/ArduPlane mavlink)
target_link_libraries(objs/AP_ESC_Telem/ArduPlane dronecan)
target_link_libraries(objs/AP_ESC_Telem/ArduPlane littlefs)

set(objs/AP_GyroFFT/ArduPlane_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs/AP_GyroFFT/ArduPlane_INCLUDES
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

include_directories(${objs/AP_GyroFFT/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_GyroFFT/ArduPlane ${objs/AP_GyroFFT/ArduPlane_SOURCES})

target_link_libraries(objs/AP_GyroFFT/ArduPlane mavlink)
target_link_libraries(objs/AP_GyroFFT/ArduPlane dronecan)
target_link_libraries(objs/AP_GyroFFT/ArduPlane littlefs)

set(objs/AP_RCTelemetry/ArduPlane_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs/AP_RCTelemetry/ArduPlane_INCLUDES
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

include_directories(${objs/AP_RCTelemetry/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_RCTelemetry/ArduPlane ${objs/AP_RCTelemetry/ArduPlane_SOURCES})

target_link_libraries(objs/AP_RCTelemetry/ArduPlane mavlink)
target_link_libraries(objs/AP_RCTelemetry/ArduPlane dronecan)
target_link_libraries(objs/AP_RCTelemetry/ArduPlane littlefs)

set(objs/AP_ExternalAHRS/ArduPlane_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs/AP_ExternalAHRS/ArduPlane_INCLUDES
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

include_directories(${objs/AP_ExternalAHRS/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_ExternalAHRS/ArduPlane ${objs/AP_ExternalAHRS/ArduPlane_SOURCES})

target_link_libraries(objs/AP_ExternalAHRS/ArduPlane mavlink)
target_link_libraries(objs/AP_ExternalAHRS/ArduPlane dronecan)
target_link_libraries(objs/AP_ExternalAHRS/ArduPlane littlefs)

set(objs/AP_TemperatureSensor/ArduPlane_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs/AP_TemperatureSensor/ArduPlane_INCLUDES
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

include_directories(${objs/AP_TemperatureSensor/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_TemperatureSensor/ArduPlane ${objs/AP_TemperatureSensor/ArduPlane_SOURCES})

target_link_libraries(objs/AP_TemperatureSensor/ArduPlane mavlink)
target_link_libraries(objs/AP_TemperatureSensor/ArduPlane dronecan)
target_link_libraries(objs/AP_TemperatureSensor/ArduPlane littlefs)

set(objs/AP_AIS/ArduPlane_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs/AP_AIS/ArduPlane_INCLUDES
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

include_directories(${objs/AP_AIS/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_AIS/ArduPlane ${objs/AP_AIS/ArduPlane_SOURCES})

target_link_libraries(objs/AP_AIS/ArduPlane mavlink)
target_link_libraries(objs/AP_AIS/ArduPlane dronecan)
target_link_libraries(objs/AP_AIS/ArduPlane littlefs)

set(objs/AP_Arming/ArduPlane_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs/AP_Arming/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Arming/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Arming/ArduPlane ${objs/AP_Arming/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Arming/ArduPlane mavlink)
target_link_libraries(objs/AP_Arming/ArduPlane dronecan)
target_link_libraries(objs/AP_Arming/ArduPlane littlefs)

set(objs/AP_Follow/ArduPlane_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs/AP_Follow/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Follow/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Follow/ArduPlane ${objs/AP_Follow/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Follow/ArduPlane mavlink)
target_link_libraries(objs/AP_Follow/ArduPlane dronecan)
target_link_libraries(objs/AP_Follow/ArduPlane littlefs)

set(objs/AP_Avoidance/ArduPlane_SOURCES
    ../libraries/AP_Avoidance/AP_Avoidance.cpp)

set(objs/AP_Avoidance/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Avoidance/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Avoidance/ArduPlane ${objs/AP_Avoidance/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Avoidance/ArduPlane mavlink)
target_link_libraries(objs/AP_Avoidance/ArduPlane dronecan)
target_link_libraries(objs/AP_Avoidance/ArduPlane littlefs)

set(objs/AC_WPNav/ArduPlane_SOURCES
    ../libraries/AC_WPNav/AC_Loiter.cpp)

set(objs/AC_WPNav/ArduPlane_INCLUDES
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

include_directories(${objs/AC_WPNav/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AC_WPNav/ArduPlane ${objs/AC_WPNav/ArduPlane_SOURCES})

target_link_libraries(objs/AC_WPNav/ArduPlane mavlink)
target_link_libraries(objs/AC_WPNav/ArduPlane dronecan)
target_link_libraries(objs/AC_WPNav/ArduPlane littlefs)

set(objs/AC_AttitudeControl/ArduPlane_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs/AC_AttitudeControl/ArduPlane_INCLUDES
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

include_directories(${objs/AC_AttitudeControl/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AC_AttitudeControl/ArduPlane ${objs/AC_AttitudeControl/ArduPlane_SOURCES})

target_link_libraries(objs/AC_AttitudeControl/ArduPlane mavlink)
target_link_libraries(objs/AC_AttitudeControl/ArduPlane dronecan)
target_link_libraries(objs/AC_AttitudeControl/ArduPlane littlefs)

set(objs/AP_Motors/ArduPlane_SOURCES
    ../libraries/AP_Motors/AP_MotorsMatrix.cpp
    ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    ../libraries/AP_Motors/AP_MotorsTri.cpp
    ../libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp)

set(objs/AP_Motors/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Motors/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Motors/ArduPlane ${objs/AP_Motors/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Motors/ArduPlane mavlink)
target_link_libraries(objs/AP_Motors/ArduPlane dronecan)
target_link_libraries(objs/AP_Motors/ArduPlane littlefs)

set(objs/AP_Landing/ArduPlane_SOURCES
    ../libraries/AP_Landing/AP_Landing_Slope.cpp)

set(objs/AP_Landing/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Landing/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Landing/ArduPlane ${objs/AP_Landing/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Landing/ArduPlane mavlink)
target_link_libraries(objs/AP_Landing/ArduPlane dronecan)
target_link_libraries(objs/AP_Landing/ArduPlane littlefs)

set(objs/AC_AutoTune/ArduPlane_SOURCES
    ../libraries/AC_AutoTune/AC_AutoTune.cpp
    ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp)

set(objs/AC_AutoTune/ArduPlane_INCLUDES
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

include_directories(${objs/AC_AutoTune/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AC_AutoTune/ArduPlane ${objs/AC_AutoTune/ArduPlane_SOURCES})

target_link_libraries(objs/AC_AutoTune/ArduPlane mavlink)
target_link_libraries(objs/AC_AutoTune/ArduPlane dronecan)
target_link_libraries(objs/AC_AutoTune/ArduPlane littlefs)

set(objs/AC_PrecLand/ArduPlane_SOURCES
    ../libraries/AC_PrecLand/AC_PrecLand.cpp)

set(objs/AC_PrecLand/ArduPlane_INCLUDES
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

include_directories(${objs/AC_PrecLand/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AC_PrecLand/ArduPlane ${objs/AC_PrecLand/ArduPlane_SOURCES})

target_link_libraries(objs/AC_PrecLand/ArduPlane mavlink)
target_link_libraries(objs/AC_PrecLand/ArduPlane dronecan)
target_link_libraries(objs/AC_PrecLand/ArduPlane littlefs)

set(objs/AP_HAL_SITL/ArduPlane_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs/AP_HAL_SITL/ArduPlane_INCLUDES
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

include_directories(${objs/AP_HAL_SITL/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_HAL_SITL/ArduPlane ${objs/AP_HAL_SITL/ArduPlane_SOURCES})

target_link_libraries(objs/AP_HAL_SITL/ArduPlane mavlink)
target_link_libraries(objs/AP_HAL_SITL/ArduPlane dronecan)
target_link_libraries(objs/AP_HAL_SITL/ArduPlane littlefs)

set(objs/SITL/ArduPlane_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs/SITL/ArduPlane_INCLUDES
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

include_directories(${objs/SITL/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/SITL/ArduPlane ${objs/SITL/ArduPlane_SOURCES})

target_link_libraries(objs/SITL/ArduPlane mavlink)
target_link_libraries(objs/SITL/ArduPlane dronecan)
target_link_libraries(objs/SITL/ArduPlane littlefs)

set(objs/AP_Scripting/ArduPlane_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs/AP_Scripting/ArduPlane_INCLUDES
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

include_directories(${objs/AP_Scripting/ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs/AP_Scripting/ArduPlane ${objs/AP_Scripting/ArduPlane_SOURCES})

target_link_libraries(objs/AP_Scripting/ArduPlane mavlink)
target_link_libraries(objs/AP_Scripting/ArduPlane dronecan)
target_link_libraries(objs/AP_Scripting/ArduPlane littlefs)

set(ArduPlane_libs_SOURCES
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

set(ArduPlane_libs_INCLUDES
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

include_directories(${ArduPlane_libs_INCLUDES})

add_library(ArduPlane_libs ${ArduPlane_libs_SOURCES})

target_link_libraries(ArduPlane_libs objs/AP_AccelCal)
target_link_libraries(ArduPlane_libs objs/AP_AccelCal/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ADC)
target_link_libraries(ArduPlane_libs objs/AP_ADC/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_AHRS)
target_link_libraries(ArduPlane_libs objs/AP_AHRS/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Airspeed)
target_link_libraries(ArduPlane_libs objs/AP_Airspeed/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Baro)
target_link_libraries(ArduPlane_libs objs/AP_Baro/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_BattMonitor)
target_link_libraries(ArduPlane_libs objs/AP_BattMonitor/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_BoardConfig)
target_link_libraries(ArduPlane_libs objs/AP_BoardConfig/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Camera)
target_link_libraries(ArduPlane_libs objs/AP_Camera/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Common)
target_link_libraries(ArduPlane_libs objs/AP_Common/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Compass)
target_link_libraries(ArduPlane_libs objs/AP_Compass/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Declination)
target_link_libraries(ArduPlane_libs objs/AP_Declination/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_GPS)
target_link_libraries(ArduPlane_libs objs/AP_GPS/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_GSOF)
target_link_libraries(ArduPlane_libs objs/AP_GSOF/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_HAL)
target_link_libraries(ArduPlane_libs objs/AP_HAL/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_HAL_Empty)
target_link_libraries(ArduPlane_libs objs/AP_HAL_Empty/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_DDS)
target_link_libraries(ArduPlane_libs objs/AP_DDS/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_InertialSensor)
target_link_libraries(ArduPlane_libs objs/AP_InertialSensor/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Math)
target_link_libraries(ArduPlane_libs objs/AP_Math/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Mission)
target_link_libraries(ArduPlane_libs objs/AP_Mission/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_DAL)
target_link_libraries(ArduPlane_libs objs/AP_DAL/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_NavEKF)
target_link_libraries(ArduPlane_libs objs/AP_NavEKF/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_NavEKF2)
target_link_libraries(ArduPlane_libs objs/AP_NavEKF2/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_NavEKF3)
target_link_libraries(ArduPlane_libs objs/AP_NavEKF3/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Notify)
target_link_libraries(ArduPlane_libs objs/AP_Notify/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_OpticalFlow)
target_link_libraries(ArduPlane_libs objs/AP_OpticalFlow/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Param)
target_link_libraries(ArduPlane_libs objs/AP_Param/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Rally)
target_link_libraries(ArduPlane_libs objs/AP_Rally/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_LightWareSerial)
target_link_libraries(ArduPlane_libs objs/AP_LightWareSerial/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RangeFinder)
target_link_libraries(ArduPlane_libs objs/AP_RangeFinder/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Scheduler)
target_link_libraries(ArduPlane_libs objs/AP_Scheduler/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_SerialManager)
target_link_libraries(ArduPlane_libs objs/AP_SerialManager/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Terrain)
target_link_libraries(ArduPlane_libs objs/AP_Terrain/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Vehicle)
target_link_libraries(ArduPlane_libs objs/AP_Vehicle/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_InternalError)
target_link_libraries(ArduPlane_libs objs/AP_InternalError/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Logger)
target_link_libraries(ArduPlane_libs objs/AP_Logger/ArduPlane)
target_link_libraries(ArduPlane_libs objs/Filter)
target_link_libraries(ArduPlane_libs objs/Filter/ArduPlane)
target_link_libraries(ArduPlane_libs objs/GCS_MAVLink)
target_link_libraries(ArduPlane_libs objs/GCS_MAVLink/ArduPlane)
target_link_libraries(ArduPlane_libs objs/RC_Channel)
target_link_libraries(ArduPlane_libs objs/RC_Channel/ArduPlane)
target_link_libraries(ArduPlane_libs objs/SRV_Channel)
target_link_libraries(ArduPlane_libs objs/SRV_Channel/ArduPlane)
target_link_libraries(ArduPlane_libs objs/StorageManager)
target_link_libraries(ArduPlane_libs objs/StorageManager/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Tuning)
target_link_libraries(ArduPlane_libs objs/AP_Tuning/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RPM)
target_link_libraries(ArduPlane_libs objs/AP_RPM/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RSSI)
target_link_libraries(ArduPlane_libs objs/AP_RSSI/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Mount)
target_link_libraries(ArduPlane_libs objs/AP_Mount/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Module)
target_link_libraries(ArduPlane_libs objs/AP_Module/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Button)
target_link_libraries(ArduPlane_libs objs/AP_Button/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ICEngine)
target_link_libraries(ArduPlane_libs objs/AP_ICEngine/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Networking)
target_link_libraries(ArduPlane_libs objs/AP_Networking/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Frsky_Telem)
target_link_libraries(ArduPlane_libs objs/AP_Frsky_Telem/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_IBus_Telem)
target_link_libraries(ArduPlane_libs objs/AP_IBus_Telem/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_FlashStorage)
target_link_libraries(ArduPlane_libs objs/AP_FlashStorage/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Relay)
target_link_libraries(ArduPlane_libs objs/AP_Relay/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ServoRelayEvents)
target_link_libraries(ArduPlane_libs objs/AP_ServoRelayEvents/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Volz_Protocol)
target_link_libraries(ArduPlane_libs objs/AP_Volz_Protocol/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_SBusOut)
target_link_libraries(ArduPlane_libs objs/AP_SBusOut/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_IOMCU)
target_link_libraries(ArduPlane_libs objs/AP_IOMCU/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Parachute)
target_link_libraries(ArduPlane_libs objs/AP_Parachute/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RAMTRON)
target_link_libraries(ArduPlane_libs objs/AP_RAMTRON/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RCProtocol)
target_link_libraries(ArduPlane_libs objs/AP_RCProtocol/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Radio)
target_link_libraries(ArduPlane_libs objs/AP_Radio/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_TempCalibration)
target_link_libraries(ArduPlane_libs objs/AP_TempCalibration/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_VisualOdom)
target_link_libraries(ArduPlane_libs objs/AP_VisualOdom/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_BLHeli)
target_link_libraries(ArduPlane_libs objs/AP_BLHeli/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ROMFS)
target_link_libraries(ArduPlane_libs objs/AP_ROMFS/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Proximity)
target_link_libraries(ArduPlane_libs objs/AP_Proximity/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Gripper)
target_link_libraries(ArduPlane_libs objs/AP_Gripper/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RTC)
target_link_libraries(ArduPlane_libs objs/AP_RTC/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_Sprayer)
target_link_libraries(ArduPlane_libs objs/AC_Sprayer/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_Fence)
target_link_libraries(ArduPlane_libs objs/AC_Fence/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_Avoidance)
target_link_libraries(ArduPlane_libs objs/AC_Avoidance/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_LandingGear)
target_link_libraries(ArduPlane_libs objs/AP_LandingGear/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RobotisServo)
target_link_libraries(ArduPlane_libs objs/AP_RobotisServo/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_NMEA_Output)
target_link_libraries(ArduPlane_libs objs/AP_NMEA_Output/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_OSD)
target_link_libraries(ArduPlane_libs objs/AP_OSD/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Filesystem)
target_link_libraries(ArduPlane_libs objs/AP_Filesystem/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ADSB)
target_link_libraries(ArduPlane_libs objs/AP_ADSB/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ADSB/sagetech-sdk)
target_link_libraries(ArduPlane_libs objs/AP_ADSB/sagetech-sdk/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_PID)
target_link_libraries(ArduPlane_libs objs/AC_PID/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_SerialLED)
target_link_libraries(ArduPlane_libs objs/AP_SerialLED/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_EFI)
target_link_libraries(ArduPlane_libs objs/AP_EFI/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Hott_Telem)
target_link_libraries(ArduPlane_libs objs/AP_Hott_Telem/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ESC_Telem)
target_link_libraries(ArduPlane_libs objs/AP_ESC_Telem/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Servo_Telem)
target_link_libraries(ArduPlane_libs objs/AP_Servo_Telem/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Stats)
target_link_libraries(ArduPlane_libs objs/AP_Stats/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_GyroFFT)
target_link_libraries(ArduPlane_libs objs/AP_GyroFFT/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RCTelemetry)
target_link_libraries(ArduPlane_libs objs/AP_RCTelemetry/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Generator)
target_link_libraries(ArduPlane_libs objs/AP_Generator/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_MSP)
target_link_libraries(ArduPlane_libs objs/AP_MSP/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_OLC)
target_link_libraries(ArduPlane_libs objs/AP_OLC/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_WheelEncoder)
target_link_libraries(ArduPlane_libs objs/AP_WheelEncoder/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ExternalAHRS)
target_link_libraries(ArduPlane_libs objs/AP_ExternalAHRS/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_VideoTX)
target_link_libraries(ArduPlane_libs objs/AP_VideoTX/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_FETtecOneWire)
target_link_libraries(ArduPlane_libs objs/AP_FETtecOneWire/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_TemperatureSensor)
target_link_libraries(ArduPlane_libs objs/AP_TemperatureSensor/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Torqeedo)
target_link_libraries(ArduPlane_libs objs/AP_Torqeedo/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_CustomRotations)
target_link_libraries(ArduPlane_libs objs/AP_CustomRotations/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_AIS)
target_link_libraries(ArduPlane_libs objs/AP_AIS/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_OpenDroneID)
target_link_libraries(ArduPlane_libs objs/AP_OpenDroneID/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_CheckFirmware)
target_link_libraries(ArduPlane_libs objs/AP_CheckFirmware/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_ExternalControl)
target_link_libraries(ArduPlane_libs objs/AP_ExternalControl/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_JSON)
target_link_libraries(ArduPlane_libs objs/AP_JSON/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Beacon)
target_link_libraries(ArduPlane_libs objs/AP_Beacon/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Arming)
target_link_libraries(ArduPlane_libs objs/AP_Arming/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_RCMapper)
target_link_libraries(ArduPlane_libs objs/AP_RCMapper/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_MultiHeap)
target_link_libraries(ArduPlane_libs objs/AP_MultiHeap/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Follow)
target_link_libraries(ArduPlane_libs objs/AP_Follow/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_CANManager)
target_link_libraries(ArduPlane_libs objs/AP_CANManager/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_KDECAN)
target_link_libraries(ArduPlane_libs objs/AP_KDECAN/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_PiccoloCAN)
target_link_libraries(ArduPlane_libs objs/AP_PiccoloCAN/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_PiccoloCAN/piccolo_protocol)
target_link_libraries(ArduPlane_libs objs/AP_PiccoloCAN/piccolo_protocol/ArduPlane)
target_link_libraries(ArduPlane_libs objs/APM_Control)
target_link_libraries(ArduPlane_libs objs/APM_Control/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_AdvancedFailsafe)
target_link_libraries(ArduPlane_libs objs/AP_AdvancedFailsafe/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Avoidance)
target_link_libraries(ArduPlane_libs objs/AP_Avoidance/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_L1_Control)
target_link_libraries(ArduPlane_libs objs/AP_L1_Control/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Navigation)
target_link_libraries(ArduPlane_libs objs/AP_Navigation/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_TECS)
target_link_libraries(ArduPlane_libs objs/AP_TECS/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_InertialNav)
target_link_libraries(ArduPlane_libs objs/AP_InertialNav/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_WPNav)
target_link_libraries(ArduPlane_libs objs/AC_WPNav/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_AttitudeControl)
target_link_libraries(ArduPlane_libs objs/AC_AttitudeControl/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Motors)
target_link_libraries(ArduPlane_libs objs/AP_Motors/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Landing)
target_link_libraries(ArduPlane_libs objs/AP_Landing/ArduPlane)
target_link_libraries(ArduPlane_libs objs/PID)
target_link_libraries(ArduPlane_libs objs/PID/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Soaring)
target_link_libraries(ArduPlane_libs objs/AP_Soaring/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_LTM_Telem)
target_link_libraries(ArduPlane_libs objs/AP_LTM_Telem/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Devo_Telem)
target_link_libraries(ArduPlane_libs objs/AP_Devo_Telem/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_AutoTune)
target_link_libraries(ArduPlane_libs objs/AC_AutoTune/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_PrecLand)
target_link_libraries(ArduPlane_libs objs/AC_PrecLand/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_IRLock)
target_link_libraries(ArduPlane_libs objs/AP_IRLock/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Quicktune)
target_link_libraries(ArduPlane_libs objs/AP_Quicktune/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_DroneCAN)
target_link_libraries(ArduPlane_libs objs/AP_DroneCAN/ArduPlane)
target_link_libraries(ArduPlane_libs objs/modules/DroneCAN/libcanard/*.c)
target_link_libraries(ArduPlane_libs objs/modules/DroneCAN/libcanard/*.c/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_HAL_SITL)
target_link_libraries(ArduPlane_libs objs/AP_HAL_SITL/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_CSVReader)
target_link_libraries(ArduPlane_libs objs/AP_CSVReader/ArduPlane)
target_link_libraries(ArduPlane_libs objs/SITL)
target_link_libraries(ArduPlane_libs objs/SITL/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Scripting)
target_link_libraries(ArduPlane_libs objs/AP_Scripting/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AP_Scripting/lua/src)
target_link_libraries(ArduPlane_libs objs/AP_Scripting/lua/src/ArduPlane)
target_link_libraries(ArduPlane_libs objs/AC_CustomControl)
target_link_libraries(ArduPlane_libs objs/AC_CustomControl/ArduPlane)

set(bin/arduplane_SOURCES
    AP_Arming_Plane.cpp
    AP_ExternalControl_Plane.cpp
    Attitude.cpp
    GCS_MAVLink_Plane.cpp
    GCS_Plane.cpp
    Log.cpp
    Parameters.cpp
    Plane.cpp
    RC_Channel_Plane.cpp
    VTOL_Assist.cpp
    afs_plane.cpp
    altitude.cpp
    avoidance_adsb.cpp
    commands.cpp
    commands_logic.cpp
    control_modes.cpp
    ekf_check.cpp
    events.cpp
    failsafe.cpp
    fence.cpp
    is_flying.cpp
    mode.cpp
    mode_LoiterAltQLand.cpp
    mode_acro.cpp
    mode_auto.cpp
    mode_autoland.cpp
    mode_autotune.cpp
    mode_avoidADSB.cpp
    mode_circle.cpp
    mode_cruise.cpp
    mode_fbwa.cpp
    mode_fbwb.cpp
    mode_guided.cpp
    mode_loiter.cpp
    mode_manual.cpp
    mode_qacro.cpp
    mode_qautotune.cpp
    mode_qhover.cpp
    mode_qland.cpp
    mode_qloiter.cpp
    mode_qrtl.cpp
    mode_qstabilize.cpp
    mode_rtl.cpp
    mode_stabilize.cpp
    mode_takeoff.cpp
    mode_thermal.cpp
    mode_training.cpp
    motor_test.cpp
    navigation.cpp
    parachute.cpp
    pullup.cpp
    qautotune.cpp
    quadplane.cpp
    radio.cpp
    reverse_thrust.cpp
    sensors.cpp
    servos.cpp
    soaring.cpp
    system.cpp
    systemid.cpp
    tailsitter.cpp
    takeoff.cpp
    tiltrotor.cpp
    tuning.cpp)

set(bin/arduplane_INCLUDES
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

include_directories(${bin/arduplane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_executable(bin/arduplane ${bin/arduplane_SOURCES})

target_link_libraries(bin/arduplane A)
target_link_libraries(bin/arduplane r)
target_link_libraries(bin/arduplane d)
target_link_libraries(bin/arduplane u)
target_link_libraries(bin/arduplane P)
target_link_libraries(bin/arduplane l)
target_link_libraries(bin/arduplane a)
target_link_libraries(bin/arduplane n)
target_link_libraries(bin/arduplane e)
target_link_libraries(bin/arduplane _)
target_link_libraries(bin/arduplane l)
target_link_libraries(bin/arduplane i)
target_link_libraries(bin/arduplane b)
target_link_libraries(bin/arduplane s)

