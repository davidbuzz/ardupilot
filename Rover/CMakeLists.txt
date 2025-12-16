
set(objs/AP_AHRS/Rover_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs/AP_AHRS/Rover_INCLUDES
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

include_directories(${objs/AP_AHRS/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_AHRS/Rover ${objs/AP_AHRS/Rover_SOURCES})

target_link_libraries(objs/AP_AHRS/Rover mavlink)
target_link_libraries(objs/AP_AHRS/Rover dronecan)
target_link_libraries(objs/AP_AHRS/Rover littlefs)

set(objs/AP_Airspeed/Rover_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs/AP_Airspeed/Rover_INCLUDES
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

include_directories(${objs/AP_Airspeed/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Airspeed/Rover ${objs/AP_Airspeed/Rover_SOURCES})

target_link_libraries(objs/AP_Airspeed/Rover mavlink)
target_link_libraries(objs/AP_Airspeed/Rover dronecan)
target_link_libraries(objs/AP_Airspeed/Rover littlefs)

set(objs/AP_Baro/Rover_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs/AP_Baro/Rover_INCLUDES
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

include_directories(${objs/AP_Baro/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Baro/Rover ${objs/AP_Baro/Rover_SOURCES})

target_link_libraries(objs/AP_Baro/Rover mavlink)
target_link_libraries(objs/AP_Baro/Rover dronecan)
target_link_libraries(objs/AP_Baro/Rover littlefs)

set(objs/AP_BattMonitor/Rover_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs/AP_BattMonitor/Rover_INCLUDES
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

include_directories(${objs/AP_BattMonitor/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_BattMonitor/Rover ${objs/AP_BattMonitor/Rover_SOURCES})

target_link_libraries(objs/AP_BattMonitor/Rover mavlink)
target_link_libraries(objs/AP_BattMonitor/Rover dronecan)
target_link_libraries(objs/AP_BattMonitor/Rover littlefs)

set(objs/AP_BoardConfig/Rover_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs/AP_BoardConfig/Rover_INCLUDES
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

include_directories(${objs/AP_BoardConfig/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_BoardConfig/Rover ${objs/AP_BoardConfig/Rover_SOURCES})

target_link_libraries(objs/AP_BoardConfig/Rover mavlink)
target_link_libraries(objs/AP_BoardConfig/Rover dronecan)
target_link_libraries(objs/AP_BoardConfig/Rover littlefs)

set(objs/AP_HAL/Rover_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs/AP_HAL/Rover_INCLUDES
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

include_directories(${objs/AP_HAL/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_HAL/Rover ${objs/AP_HAL/Rover_SOURCES})

target_link_libraries(objs/AP_HAL/Rover mavlink)
target_link_libraries(objs/AP_HAL/Rover dronecan)
target_link_libraries(objs/AP_HAL/Rover littlefs)

set(objs/AP_InertialSensor/Rover_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs/AP_InertialSensor/Rover_INCLUDES
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

include_directories(${objs/AP_InertialSensor/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_InertialSensor/Rover ${objs/AP_InertialSensor/Rover_SOURCES})

target_link_libraries(objs/AP_InertialSensor/Rover mavlink)
target_link_libraries(objs/AP_InertialSensor/Rover dronecan)
target_link_libraries(objs/AP_InertialSensor/Rover littlefs)

set(objs/AP_Math/Rover_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs/AP_Math/Rover_INCLUDES
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

include_directories(${objs/AP_Math/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Math/Rover ${objs/AP_Math/Rover_SOURCES})

target_link_libraries(objs/AP_Math/Rover mavlink)
target_link_libraries(objs/AP_Math/Rover dronecan)
target_link_libraries(objs/AP_Math/Rover littlefs)

set(objs/AP_Mission/Rover_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs/AP_Mission/Rover_INCLUDES
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

include_directories(${objs/AP_Mission/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Mission/Rover ${objs/AP_Mission/Rover_SOURCES})

target_link_libraries(objs/AP_Mission/Rover mavlink)
target_link_libraries(objs/AP_Mission/Rover dronecan)
target_link_libraries(objs/AP_Mission/Rover littlefs)

set(objs/AP_DAL/Rover_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs/AP_DAL/Rover_INCLUDES
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

include_directories(${objs/AP_DAL/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_DAL/Rover ${objs/AP_DAL/Rover_SOURCES})

target_link_libraries(objs/AP_DAL/Rover mavlink)
target_link_libraries(objs/AP_DAL/Rover dronecan)
target_link_libraries(objs/AP_DAL/Rover littlefs)

set(objs/AP_NavEKF2/Rover_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs/AP_NavEKF2/Rover_INCLUDES
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

include_directories(${objs/AP_NavEKF2/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_NavEKF2/Rover ${objs/AP_NavEKF2/Rover_SOURCES})

target_link_libraries(objs/AP_NavEKF2/Rover mavlink)
target_link_libraries(objs/AP_NavEKF2/Rover dronecan)
target_link_libraries(objs/AP_NavEKF2/Rover littlefs)

set(objs/AP_NavEKF3/Rover_SOURCES
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

set(objs/AP_NavEKF3/Rover_INCLUDES
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

include_directories(${objs/AP_NavEKF3/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_NavEKF3/Rover ${objs/AP_NavEKF3/Rover_SOURCES})

target_link_libraries(objs/AP_NavEKF3/Rover mavlink)
target_link_libraries(objs/AP_NavEKF3/Rover dronecan)
target_link_libraries(objs/AP_NavEKF3/Rover littlefs)

set(objs/AP_Rally/Rover_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs/AP_Rally/Rover_INCLUDES
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

include_directories(${objs/AP_Rally/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Rally/Rover ${objs/AP_Rally/Rover_SOURCES})

target_link_libraries(objs/AP_Rally/Rover mavlink)
target_link_libraries(objs/AP_Rally/Rover dronecan)
target_link_libraries(objs/AP_Rally/Rover littlefs)

set(objs/AP_Scheduler/Rover_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs/AP_Scheduler/Rover_INCLUDES
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

include_directories(${objs/AP_Scheduler/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Scheduler/Rover ${objs/AP_Scheduler/Rover_SOURCES})

target_link_libraries(objs/AP_Scheduler/Rover mavlink)
target_link_libraries(objs/AP_Scheduler/Rover dronecan)
target_link_libraries(objs/AP_Scheduler/Rover littlefs)

set(objs/AP_Terrain/Rover_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs/AP_Terrain/Rover_INCLUDES
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

include_directories(${objs/AP_Terrain/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Terrain/Rover ${objs/AP_Terrain/Rover_SOURCES})

target_link_libraries(objs/AP_Terrain/Rover mavlink)
target_link_libraries(objs/AP_Terrain/Rover dronecan)
target_link_libraries(objs/AP_Terrain/Rover littlefs)

set(objs/AP_Vehicle/Rover_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs/AP_Vehicle/Rover_INCLUDES
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

include_directories(${objs/AP_Vehicle/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Vehicle/Rover ${objs/AP_Vehicle/Rover_SOURCES})

target_link_libraries(objs/AP_Vehicle/Rover mavlink)
target_link_libraries(objs/AP_Vehicle/Rover dronecan)
target_link_libraries(objs/AP_Vehicle/Rover littlefs)

set(objs/AP_Logger/Rover_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs/AP_Logger/Rover_INCLUDES
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

include_directories(${objs/AP_Logger/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Logger/Rover ${objs/AP_Logger/Rover_SOURCES})

target_link_libraries(objs/AP_Logger/Rover mavlink)
target_link_libraries(objs/AP_Logger/Rover dronecan)
target_link_libraries(objs/AP_Logger/Rover littlefs)

set(objs/Filter/Rover_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs/Filter/Rover_INCLUDES
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

include_directories(${objs/Filter/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/Filter/Rover ${objs/Filter/Rover_SOURCES})

target_link_libraries(objs/Filter/Rover mavlink)
target_link_libraries(objs/Filter/Rover dronecan)
target_link_libraries(objs/Filter/Rover littlefs)

set(objs/GCS_MAVLink/Rover_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs/GCS_MAVLink/Rover_INCLUDES
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

include_directories(${objs/GCS_MAVLink/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/GCS_MAVLink/Rover ${objs/GCS_MAVLink/Rover_SOURCES})

target_link_libraries(objs/GCS_MAVLink/Rover mavlink)
target_link_libraries(objs/GCS_MAVLink/Rover dronecan)
target_link_libraries(objs/GCS_MAVLink/Rover littlefs)

set(objs/RC_Channel/Rover_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs/RC_Channel/Rover_INCLUDES
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

include_directories(${objs/RC_Channel/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/RC_Channel/Rover ${objs/RC_Channel/Rover_SOURCES})

target_link_libraries(objs/RC_Channel/Rover mavlink)
target_link_libraries(objs/RC_Channel/Rover dronecan)
target_link_libraries(objs/RC_Channel/Rover littlefs)

set(objs/StorageManager/Rover_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs/StorageManager/Rover_INCLUDES
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

include_directories(${objs/StorageManager/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/StorageManager/Rover ${objs/StorageManager/Rover_SOURCES})

target_link_libraries(objs/StorageManager/Rover mavlink)
target_link_libraries(objs/StorageManager/Rover dronecan)
target_link_libraries(objs/StorageManager/Rover littlefs)

set(objs/AP_Frsky_Telem/Rover_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs/AP_Frsky_Telem/Rover_INCLUDES
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

include_directories(${objs/AP_Frsky_Telem/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Frsky_Telem/Rover ${objs/AP_Frsky_Telem/Rover_SOURCES})

target_link_libraries(objs/AP_Frsky_Telem/Rover mavlink)
target_link_libraries(objs/AP_Frsky_Telem/Rover dronecan)
target_link_libraries(objs/AP_Frsky_Telem/Rover littlefs)

set(objs/AP_IBus_Telem/Rover_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs/AP_IBus_Telem/Rover_INCLUDES
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

include_directories(${objs/AP_IBus_Telem/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_IBus_Telem/Rover ${objs/AP_IBus_Telem/Rover_SOURCES})

target_link_libraries(objs/AP_IBus_Telem/Rover mavlink)
target_link_libraries(objs/AP_IBus_Telem/Rover dronecan)
target_link_libraries(objs/AP_IBus_Telem/Rover littlefs)

set(objs/AP_Relay/Rover_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs/AP_Relay/Rover_INCLUDES
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

include_directories(${objs/AP_Relay/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Relay/Rover ${objs/AP_Relay/Rover_SOURCES})

target_link_libraries(objs/AP_Relay/Rover mavlink)
target_link_libraries(objs/AP_Relay/Rover dronecan)
target_link_libraries(objs/AP_Relay/Rover littlefs)

set(objs/AP_RCProtocol/Rover_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs/AP_RCProtocol/Rover_INCLUDES
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

include_directories(${objs/AP_RCProtocol/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_RCProtocol/Rover ${objs/AP_RCProtocol/Rover_SOURCES})

target_link_libraries(objs/AP_RCProtocol/Rover mavlink)
target_link_libraries(objs/AP_RCProtocol/Rover dronecan)
target_link_libraries(objs/AP_RCProtocol/Rover littlefs)

set(objs/AP_BLHeli/Rover_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs/AP_BLHeli/Rover_INCLUDES
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

include_directories(${objs/AP_BLHeli/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_BLHeli/Rover ${objs/AP_BLHeli/Rover_SOURCES})

target_link_libraries(objs/AP_BLHeli/Rover mavlink)
target_link_libraries(objs/AP_BLHeli/Rover dronecan)
target_link_libraries(objs/AP_BLHeli/Rover littlefs)

set(objs/AP_Proximity/Rover_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs/AP_Proximity/Rover_INCLUDES
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

include_directories(${objs/AP_Proximity/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Proximity/Rover ${objs/AP_Proximity/Rover_SOURCES})

target_link_libraries(objs/AP_Proximity/Rover mavlink)
target_link_libraries(objs/AP_Proximity/Rover dronecan)
target_link_libraries(objs/AP_Proximity/Rover littlefs)

set(objs/AC_Fence/Rover_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs/AC_Fence/Rover_INCLUDES
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

include_directories(${objs/AC_Fence/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AC_Fence/Rover ${objs/AC_Fence/Rover_SOURCES})

target_link_libraries(objs/AC_Fence/Rover mavlink)
target_link_libraries(objs/AC_Fence/Rover dronecan)
target_link_libraries(objs/AC_Fence/Rover littlefs)

set(objs/AC_Avoidance/Rover_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs/AC_Avoidance/Rover_INCLUDES
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

include_directories(${objs/AC_Avoidance/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AC_Avoidance/Rover ${objs/AC_Avoidance/Rover_SOURCES})

target_link_libraries(objs/AC_Avoidance/Rover mavlink)
target_link_libraries(objs/AC_Avoidance/Rover dronecan)
target_link_libraries(objs/AC_Avoidance/Rover littlefs)

set(objs/AP_LandingGear/Rover_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs/AP_LandingGear/Rover_INCLUDES
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

include_directories(${objs/AP_LandingGear/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_LandingGear/Rover ${objs/AP_LandingGear/Rover_SOURCES})

target_link_libraries(objs/AP_LandingGear/Rover mavlink)
target_link_libraries(objs/AP_LandingGear/Rover dronecan)
target_link_libraries(objs/AP_LandingGear/Rover littlefs)

set(objs/AP_OSD/Rover_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs/AP_OSD/Rover_INCLUDES
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

include_directories(${objs/AP_OSD/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_OSD/Rover ${objs/AP_OSD/Rover_SOURCES})

target_link_libraries(objs/AP_OSD/Rover mavlink)
target_link_libraries(objs/AP_OSD/Rover dronecan)
target_link_libraries(objs/AP_OSD/Rover littlefs)

set(objs/AP_Filesystem/Rover_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs/AP_Filesystem/Rover_INCLUDES
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

include_directories(${objs/AP_Filesystem/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Filesystem/Rover ${objs/AP_Filesystem/Rover_SOURCES})

target_link_libraries(objs/AP_Filesystem/Rover mavlink)
target_link_libraries(objs/AP_Filesystem/Rover dronecan)
target_link_libraries(objs/AP_Filesystem/Rover littlefs)

set(objs/AP_ADSB/Rover_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs/AP_ADSB/Rover_INCLUDES
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

include_directories(${objs/AP_ADSB/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_ADSB/Rover ${objs/AP_ADSB/Rover_SOURCES})

target_link_libraries(objs/AP_ADSB/Rover mavlink)
target_link_libraries(objs/AP_ADSB/Rover dronecan)
target_link_libraries(objs/AP_ADSB/Rover littlefs)

set(objs/AP_ESC_Telem/Rover_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs/AP_ESC_Telem/Rover_INCLUDES
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

include_directories(${objs/AP_ESC_Telem/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_ESC_Telem/Rover ${objs/AP_ESC_Telem/Rover_SOURCES})

target_link_libraries(objs/AP_ESC_Telem/Rover mavlink)
target_link_libraries(objs/AP_ESC_Telem/Rover dronecan)
target_link_libraries(objs/AP_ESC_Telem/Rover littlefs)

set(objs/AP_GyroFFT/Rover_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs/AP_GyroFFT/Rover_INCLUDES
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

include_directories(${objs/AP_GyroFFT/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_GyroFFT/Rover ${objs/AP_GyroFFT/Rover_SOURCES})

target_link_libraries(objs/AP_GyroFFT/Rover mavlink)
target_link_libraries(objs/AP_GyroFFT/Rover dronecan)
target_link_libraries(objs/AP_GyroFFT/Rover littlefs)

set(objs/AP_RCTelemetry/Rover_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs/AP_RCTelemetry/Rover_INCLUDES
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

include_directories(${objs/AP_RCTelemetry/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_RCTelemetry/Rover ${objs/AP_RCTelemetry/Rover_SOURCES})

target_link_libraries(objs/AP_RCTelemetry/Rover mavlink)
target_link_libraries(objs/AP_RCTelemetry/Rover dronecan)
target_link_libraries(objs/AP_RCTelemetry/Rover littlefs)

set(objs/AP_ExternalAHRS/Rover_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs/AP_ExternalAHRS/Rover_INCLUDES
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

include_directories(${objs/AP_ExternalAHRS/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_ExternalAHRS/Rover ${objs/AP_ExternalAHRS/Rover_SOURCES})

target_link_libraries(objs/AP_ExternalAHRS/Rover mavlink)
target_link_libraries(objs/AP_ExternalAHRS/Rover dronecan)
target_link_libraries(objs/AP_ExternalAHRS/Rover littlefs)

set(objs/AP_TemperatureSensor/Rover_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs/AP_TemperatureSensor/Rover_INCLUDES
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

include_directories(${objs/AP_TemperatureSensor/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_TemperatureSensor/Rover ${objs/AP_TemperatureSensor/Rover_SOURCES})

target_link_libraries(objs/AP_TemperatureSensor/Rover mavlink)
target_link_libraries(objs/AP_TemperatureSensor/Rover dronecan)
target_link_libraries(objs/AP_TemperatureSensor/Rover littlefs)

set(objs/AP_AIS/Rover_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs/AP_AIS/Rover_INCLUDES
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

include_directories(${objs/AP_AIS/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_AIS/Rover ${objs/AP_AIS/Rover_SOURCES})

target_link_libraries(objs/AP_AIS/Rover mavlink)
target_link_libraries(objs/AP_AIS/Rover dronecan)
target_link_libraries(objs/AP_AIS/Rover littlefs)

set(objs/AP_Arming/Rover_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs/AP_Arming/Rover_INCLUDES
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

include_directories(${objs/AP_Arming/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Arming/Rover ${objs/AP_Arming/Rover_SOURCES})

target_link_libraries(objs/AP_Arming/Rover mavlink)
target_link_libraries(objs/AP_Arming/Rover dronecan)
target_link_libraries(objs/AP_Arming/Rover littlefs)

set(objs/AP_Follow/Rover_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs/AP_Follow/Rover_INCLUDES
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

include_directories(${objs/AP_Follow/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Follow/Rover ${objs/AP_Follow/Rover_SOURCES})

target_link_libraries(objs/AP_Follow/Rover mavlink)
target_link_libraries(objs/AP_Follow/Rover dronecan)
target_link_libraries(objs/AP_Follow/Rover littlefs)

set(objs/AC_AttitudeControl/Rover_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs/AC_AttitudeControl/Rover_INCLUDES
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

include_directories(${objs/AC_AttitudeControl/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AC_AttitudeControl/Rover ${objs/AC_AttitudeControl/Rover_SOURCES})

target_link_libraries(objs/AC_AttitudeControl/Rover mavlink)
target_link_libraries(objs/AC_AttitudeControl/Rover dronecan)
target_link_libraries(objs/AC_AttitudeControl/Rover littlefs)

set(objs/AC_PrecLand/Rover_SOURCES
    ../libraries/AC_PrecLand/AC_PrecLand.cpp)

set(objs/AC_PrecLand/Rover_INCLUDES
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

include_directories(${objs/AC_PrecLand/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AC_PrecLand/Rover ${objs/AC_PrecLand/Rover_SOURCES})

target_link_libraries(objs/AC_PrecLand/Rover mavlink)
target_link_libraries(objs/AC_PrecLand/Rover dronecan)
target_link_libraries(objs/AC_PrecLand/Rover littlefs)

set(objs/AP_HAL_SITL/Rover_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs/AP_HAL_SITL/Rover_INCLUDES
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

include_directories(${objs/AP_HAL_SITL/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_HAL_SITL/Rover ${objs/AP_HAL_SITL/Rover_SOURCES})

target_link_libraries(objs/AP_HAL_SITL/Rover mavlink)
target_link_libraries(objs/AP_HAL_SITL/Rover dronecan)
target_link_libraries(objs/AP_HAL_SITL/Rover littlefs)

set(objs/SITL/Rover_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs/SITL/Rover_INCLUDES
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

include_directories(${objs/SITL/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/SITL/Rover ${objs/SITL/Rover_SOURCES})

target_link_libraries(objs/SITL/Rover mavlink)
target_link_libraries(objs/SITL/Rover dronecan)
target_link_libraries(objs/SITL/Rover littlefs)

set(objs/AP_Scripting/Rover_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs/AP_Scripting/Rover_INCLUDES
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

include_directories(${objs/AP_Scripting/Rover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_library(objs/AP_Scripting/Rover ${objs/AP_Scripting/Rover_SOURCES})

target_link_libraries(objs/AP_Scripting/Rover mavlink)
target_link_libraries(objs/AP_Scripting/Rover dronecan)
target_link_libraries(objs/AP_Scripting/Rover littlefs)

set(Rover_libs_SOURCES
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

set(Rover_libs_INCLUDES
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

include_directories(${Rover_libs_INCLUDES})

add_library(Rover_libs ${Rover_libs_SOURCES})

target_link_libraries(Rover_libs objs/AP_AccelCal)
target_link_libraries(Rover_libs objs/AP_AccelCal/Rover)
target_link_libraries(Rover_libs objs/AP_ADC)
target_link_libraries(Rover_libs objs/AP_ADC/Rover)
target_link_libraries(Rover_libs objs/AP_AHRS)
target_link_libraries(Rover_libs objs/AP_AHRS/Rover)
target_link_libraries(Rover_libs objs/AP_Airspeed)
target_link_libraries(Rover_libs objs/AP_Airspeed/Rover)
target_link_libraries(Rover_libs objs/AP_Baro)
target_link_libraries(Rover_libs objs/AP_Baro/Rover)
target_link_libraries(Rover_libs objs/AP_BattMonitor)
target_link_libraries(Rover_libs objs/AP_BattMonitor/Rover)
target_link_libraries(Rover_libs objs/AP_BoardConfig)
target_link_libraries(Rover_libs objs/AP_BoardConfig/Rover)
target_link_libraries(Rover_libs objs/AP_Camera)
target_link_libraries(Rover_libs objs/AP_Camera/Rover)
target_link_libraries(Rover_libs objs/AP_Common)
target_link_libraries(Rover_libs objs/AP_Common/Rover)
target_link_libraries(Rover_libs objs/AP_Compass)
target_link_libraries(Rover_libs objs/AP_Compass/Rover)
target_link_libraries(Rover_libs objs/AP_Declination)
target_link_libraries(Rover_libs objs/AP_Declination/Rover)
target_link_libraries(Rover_libs objs/AP_GPS)
target_link_libraries(Rover_libs objs/AP_GPS/Rover)
target_link_libraries(Rover_libs objs/AP_GSOF)
target_link_libraries(Rover_libs objs/AP_GSOF/Rover)
target_link_libraries(Rover_libs objs/AP_HAL)
target_link_libraries(Rover_libs objs/AP_HAL/Rover)
target_link_libraries(Rover_libs objs/AP_HAL_Empty)
target_link_libraries(Rover_libs objs/AP_HAL_Empty/Rover)
target_link_libraries(Rover_libs objs/AP_DDS)
target_link_libraries(Rover_libs objs/AP_DDS/Rover)
target_link_libraries(Rover_libs objs/AP_InertialSensor)
target_link_libraries(Rover_libs objs/AP_InertialSensor/Rover)
target_link_libraries(Rover_libs objs/AP_Math)
target_link_libraries(Rover_libs objs/AP_Math/Rover)
target_link_libraries(Rover_libs objs/AP_Mission)
target_link_libraries(Rover_libs objs/AP_Mission/Rover)
target_link_libraries(Rover_libs objs/AP_DAL)
target_link_libraries(Rover_libs objs/AP_DAL/Rover)
target_link_libraries(Rover_libs objs/AP_NavEKF)
target_link_libraries(Rover_libs objs/AP_NavEKF/Rover)
target_link_libraries(Rover_libs objs/AP_NavEKF2)
target_link_libraries(Rover_libs objs/AP_NavEKF2/Rover)
target_link_libraries(Rover_libs objs/AP_NavEKF3)
target_link_libraries(Rover_libs objs/AP_NavEKF3/Rover)
target_link_libraries(Rover_libs objs/AP_Notify)
target_link_libraries(Rover_libs objs/AP_Notify/Rover)
target_link_libraries(Rover_libs objs/AP_OpticalFlow)
target_link_libraries(Rover_libs objs/AP_OpticalFlow/Rover)
target_link_libraries(Rover_libs objs/AP_Param)
target_link_libraries(Rover_libs objs/AP_Param/Rover)
target_link_libraries(Rover_libs objs/AP_Rally)
target_link_libraries(Rover_libs objs/AP_Rally/Rover)
target_link_libraries(Rover_libs objs/AP_LightWareSerial)
target_link_libraries(Rover_libs objs/AP_LightWareSerial/Rover)
target_link_libraries(Rover_libs objs/AP_RangeFinder)
target_link_libraries(Rover_libs objs/AP_RangeFinder/Rover)
target_link_libraries(Rover_libs objs/AP_Scheduler)
target_link_libraries(Rover_libs objs/AP_Scheduler/Rover)
target_link_libraries(Rover_libs objs/AP_SerialManager)
target_link_libraries(Rover_libs objs/AP_SerialManager/Rover)
target_link_libraries(Rover_libs objs/AP_Terrain)
target_link_libraries(Rover_libs objs/AP_Terrain/Rover)
target_link_libraries(Rover_libs objs/AP_Vehicle)
target_link_libraries(Rover_libs objs/AP_Vehicle/Rover)
target_link_libraries(Rover_libs objs/AP_InternalError)
target_link_libraries(Rover_libs objs/AP_InternalError/Rover)
target_link_libraries(Rover_libs objs/AP_Logger)
target_link_libraries(Rover_libs objs/AP_Logger/Rover)
target_link_libraries(Rover_libs objs/Filter)
target_link_libraries(Rover_libs objs/Filter/Rover)
target_link_libraries(Rover_libs objs/GCS_MAVLink)
target_link_libraries(Rover_libs objs/GCS_MAVLink/Rover)
target_link_libraries(Rover_libs objs/RC_Channel)
target_link_libraries(Rover_libs objs/RC_Channel/Rover)
target_link_libraries(Rover_libs objs/SRV_Channel)
target_link_libraries(Rover_libs objs/SRV_Channel/Rover)
target_link_libraries(Rover_libs objs/StorageManager)
target_link_libraries(Rover_libs objs/StorageManager/Rover)
target_link_libraries(Rover_libs objs/AP_Tuning)
target_link_libraries(Rover_libs objs/AP_Tuning/Rover)
target_link_libraries(Rover_libs objs/AP_RPM)
target_link_libraries(Rover_libs objs/AP_RPM/Rover)
target_link_libraries(Rover_libs objs/AP_RSSI)
target_link_libraries(Rover_libs objs/AP_RSSI/Rover)
target_link_libraries(Rover_libs objs/AP_Mount)
target_link_libraries(Rover_libs objs/AP_Mount/Rover)
target_link_libraries(Rover_libs objs/AP_Module)
target_link_libraries(Rover_libs objs/AP_Module/Rover)
target_link_libraries(Rover_libs objs/AP_Button)
target_link_libraries(Rover_libs objs/AP_Button/Rover)
target_link_libraries(Rover_libs objs/AP_ICEngine)
target_link_libraries(Rover_libs objs/AP_ICEngine/Rover)
target_link_libraries(Rover_libs objs/AP_Networking)
target_link_libraries(Rover_libs objs/AP_Networking/Rover)
target_link_libraries(Rover_libs objs/AP_Frsky_Telem)
target_link_libraries(Rover_libs objs/AP_Frsky_Telem/Rover)
target_link_libraries(Rover_libs objs/AP_IBus_Telem)
target_link_libraries(Rover_libs objs/AP_IBus_Telem/Rover)
target_link_libraries(Rover_libs objs/AP_FlashStorage)
target_link_libraries(Rover_libs objs/AP_FlashStorage/Rover)
target_link_libraries(Rover_libs objs/AP_Relay)
target_link_libraries(Rover_libs objs/AP_Relay/Rover)
target_link_libraries(Rover_libs objs/AP_ServoRelayEvents)
target_link_libraries(Rover_libs objs/AP_ServoRelayEvents/Rover)
target_link_libraries(Rover_libs objs/AP_Volz_Protocol)
target_link_libraries(Rover_libs objs/AP_Volz_Protocol/Rover)
target_link_libraries(Rover_libs objs/AP_SBusOut)
target_link_libraries(Rover_libs objs/AP_SBusOut/Rover)
target_link_libraries(Rover_libs objs/AP_IOMCU)
target_link_libraries(Rover_libs objs/AP_IOMCU/Rover)
target_link_libraries(Rover_libs objs/AP_Parachute)
target_link_libraries(Rover_libs objs/AP_Parachute/Rover)
target_link_libraries(Rover_libs objs/AP_RAMTRON)
target_link_libraries(Rover_libs objs/AP_RAMTRON/Rover)
target_link_libraries(Rover_libs objs/AP_RCProtocol)
target_link_libraries(Rover_libs objs/AP_RCProtocol/Rover)
target_link_libraries(Rover_libs objs/AP_Radio)
target_link_libraries(Rover_libs objs/AP_Radio/Rover)
target_link_libraries(Rover_libs objs/AP_TempCalibration)
target_link_libraries(Rover_libs objs/AP_TempCalibration/Rover)
target_link_libraries(Rover_libs objs/AP_VisualOdom)
target_link_libraries(Rover_libs objs/AP_VisualOdom/Rover)
target_link_libraries(Rover_libs objs/AP_BLHeli)
target_link_libraries(Rover_libs objs/AP_BLHeli/Rover)
target_link_libraries(Rover_libs objs/AP_ROMFS)
target_link_libraries(Rover_libs objs/AP_ROMFS/Rover)
target_link_libraries(Rover_libs objs/AP_Proximity)
target_link_libraries(Rover_libs objs/AP_Proximity/Rover)
target_link_libraries(Rover_libs objs/AP_Gripper)
target_link_libraries(Rover_libs objs/AP_Gripper/Rover)
target_link_libraries(Rover_libs objs/AP_RTC)
target_link_libraries(Rover_libs objs/AP_RTC/Rover)
target_link_libraries(Rover_libs objs/AC_Sprayer)
target_link_libraries(Rover_libs objs/AC_Sprayer/Rover)
target_link_libraries(Rover_libs objs/AC_Fence)
target_link_libraries(Rover_libs objs/AC_Fence/Rover)
target_link_libraries(Rover_libs objs/AC_Avoidance)
target_link_libraries(Rover_libs objs/AC_Avoidance/Rover)
target_link_libraries(Rover_libs objs/AP_LandingGear)
target_link_libraries(Rover_libs objs/AP_LandingGear/Rover)
target_link_libraries(Rover_libs objs/AP_RobotisServo)
target_link_libraries(Rover_libs objs/AP_RobotisServo/Rover)
target_link_libraries(Rover_libs objs/AP_NMEA_Output)
target_link_libraries(Rover_libs objs/AP_NMEA_Output/Rover)
target_link_libraries(Rover_libs objs/AP_OSD)
target_link_libraries(Rover_libs objs/AP_OSD/Rover)
target_link_libraries(Rover_libs objs/AP_Filesystem)
target_link_libraries(Rover_libs objs/AP_Filesystem/Rover)
target_link_libraries(Rover_libs objs/AP_ADSB)
target_link_libraries(Rover_libs objs/AP_ADSB/Rover)
target_link_libraries(Rover_libs objs/AP_ADSB/sagetech-sdk)
target_link_libraries(Rover_libs objs/AP_ADSB/sagetech-sdk/Rover)
target_link_libraries(Rover_libs objs/AC_PID)
target_link_libraries(Rover_libs objs/AC_PID/Rover)
target_link_libraries(Rover_libs objs/AP_SerialLED)
target_link_libraries(Rover_libs objs/AP_SerialLED/Rover)
target_link_libraries(Rover_libs objs/AP_EFI)
target_link_libraries(Rover_libs objs/AP_EFI/Rover)
target_link_libraries(Rover_libs objs/AP_Hott_Telem)
target_link_libraries(Rover_libs objs/AP_Hott_Telem/Rover)
target_link_libraries(Rover_libs objs/AP_ESC_Telem)
target_link_libraries(Rover_libs objs/AP_ESC_Telem/Rover)
target_link_libraries(Rover_libs objs/AP_Servo_Telem)
target_link_libraries(Rover_libs objs/AP_Servo_Telem/Rover)
target_link_libraries(Rover_libs objs/AP_Stats)
target_link_libraries(Rover_libs objs/AP_Stats/Rover)
target_link_libraries(Rover_libs objs/AP_GyroFFT)
target_link_libraries(Rover_libs objs/AP_GyroFFT/Rover)
target_link_libraries(Rover_libs objs/AP_RCTelemetry)
target_link_libraries(Rover_libs objs/AP_RCTelemetry/Rover)
target_link_libraries(Rover_libs objs/AP_Generator)
target_link_libraries(Rover_libs objs/AP_Generator/Rover)
target_link_libraries(Rover_libs objs/AP_MSP)
target_link_libraries(Rover_libs objs/AP_MSP/Rover)
target_link_libraries(Rover_libs objs/AP_OLC)
target_link_libraries(Rover_libs objs/AP_OLC/Rover)
target_link_libraries(Rover_libs objs/AP_WheelEncoder)
target_link_libraries(Rover_libs objs/AP_WheelEncoder/Rover)
target_link_libraries(Rover_libs objs/AP_ExternalAHRS)
target_link_libraries(Rover_libs objs/AP_ExternalAHRS/Rover)
target_link_libraries(Rover_libs objs/AP_VideoTX)
target_link_libraries(Rover_libs objs/AP_VideoTX/Rover)
target_link_libraries(Rover_libs objs/AP_FETtecOneWire)
target_link_libraries(Rover_libs objs/AP_FETtecOneWire/Rover)
target_link_libraries(Rover_libs objs/AP_TemperatureSensor)
target_link_libraries(Rover_libs objs/AP_TemperatureSensor/Rover)
target_link_libraries(Rover_libs objs/AP_Torqeedo)
target_link_libraries(Rover_libs objs/AP_Torqeedo/Rover)
target_link_libraries(Rover_libs objs/AP_CustomRotations)
target_link_libraries(Rover_libs objs/AP_CustomRotations/Rover)
target_link_libraries(Rover_libs objs/AP_AIS)
target_link_libraries(Rover_libs objs/AP_AIS/Rover)
target_link_libraries(Rover_libs objs/AP_OpenDroneID)
target_link_libraries(Rover_libs objs/AP_OpenDroneID/Rover)
target_link_libraries(Rover_libs objs/AP_CheckFirmware)
target_link_libraries(Rover_libs objs/AP_CheckFirmware/Rover)
target_link_libraries(Rover_libs objs/AP_ExternalControl)
target_link_libraries(Rover_libs objs/AP_ExternalControl/Rover)
target_link_libraries(Rover_libs objs/AP_JSON)
target_link_libraries(Rover_libs objs/AP_JSON/Rover)
target_link_libraries(Rover_libs objs/AP_Beacon)
target_link_libraries(Rover_libs objs/AP_Beacon/Rover)
target_link_libraries(Rover_libs objs/AP_Arming)
target_link_libraries(Rover_libs objs/AP_Arming/Rover)
target_link_libraries(Rover_libs objs/AP_RCMapper)
target_link_libraries(Rover_libs objs/AP_RCMapper/Rover)
target_link_libraries(Rover_libs objs/AP_MultiHeap)
target_link_libraries(Rover_libs objs/AP_MultiHeap/Rover)
target_link_libraries(Rover_libs objs/AP_Follow)
target_link_libraries(Rover_libs objs/AP_Follow/Rover)
target_link_libraries(Rover_libs objs/AP_CANManager)
target_link_libraries(Rover_libs objs/AP_CANManager/Rover)
target_link_libraries(Rover_libs objs/AP_KDECAN)
target_link_libraries(Rover_libs objs/AP_KDECAN/Rover)
target_link_libraries(Rover_libs objs/AP_PiccoloCAN)
target_link_libraries(Rover_libs objs/AP_PiccoloCAN/Rover)
target_link_libraries(Rover_libs objs/AP_PiccoloCAN/piccolo_protocol)
target_link_libraries(Rover_libs objs/AP_PiccoloCAN/piccolo_protocol/Rover)
target_link_libraries(Rover_libs objs/APM_Control)
target_link_libraries(Rover_libs objs/APM_Control/Rover)
target_link_libraries(Rover_libs objs/AP_Navigation)
target_link_libraries(Rover_libs objs/AP_Navigation/Rover)
target_link_libraries(Rover_libs objs/AR_WPNav)
target_link_libraries(Rover_libs objs/AR_WPNav/Rover)
target_link_libraries(Rover_libs objs/AP_AdvancedFailsafe)
target_link_libraries(Rover_libs objs/AP_AdvancedFailsafe/Rover)
target_link_libraries(Rover_libs objs/AP_SmartRTL)
target_link_libraries(Rover_libs objs/AP_SmartRTL/Rover)
target_link_libraries(Rover_libs objs/AC_AttitudeControl)
target_link_libraries(Rover_libs objs/AC_AttitudeControl/Rover)
target_link_libraries(Rover_libs objs/AP_LTM_Telem)
target_link_libraries(Rover_libs objs/AP_LTM_Telem/Rover)
target_link_libraries(Rover_libs objs/AP_Devo_Telem)
target_link_libraries(Rover_libs objs/AP_Devo_Telem/Rover)
target_link_libraries(Rover_libs objs/AP_WindVane)
target_link_libraries(Rover_libs objs/AP_WindVane/Rover)
target_link_libraries(Rover_libs objs/AR_Motors)
target_link_libraries(Rover_libs objs/AR_Motors/Rover)
target_link_libraries(Rover_libs objs/AC_PrecLand)
target_link_libraries(Rover_libs objs/AC_PrecLand/Rover)
target_link_libraries(Rover_libs objs/AP_IRLock)
target_link_libraries(Rover_libs objs/AP_IRLock/Rover)
target_link_libraries(Rover_libs objs/AP_DroneCAN)
target_link_libraries(Rover_libs objs/AP_DroneCAN/Rover)
target_link_libraries(Rover_libs objs/modules/DroneCAN/libcanard/*.c)
target_link_libraries(Rover_libs objs/modules/DroneCAN/libcanard/*.c/Rover)
target_link_libraries(Rover_libs objs/AP_HAL_SITL)
target_link_libraries(Rover_libs objs/AP_HAL_SITL/Rover)
target_link_libraries(Rover_libs objs/AP_CSVReader)
target_link_libraries(Rover_libs objs/AP_CSVReader/Rover)
target_link_libraries(Rover_libs objs/SITL)
target_link_libraries(Rover_libs objs/SITL/Rover)
target_link_libraries(Rover_libs objs/AP_Scripting)
target_link_libraries(Rover_libs objs/AP_Scripting/Rover)
target_link_libraries(Rover_libs objs/AP_Scripting/lua/src)
target_link_libraries(Rover_libs objs/AP_Scripting/lua/src/Rover)
target_link_libraries(Rover_libs objs/AC_CustomControl)
target_link_libraries(Rover_libs objs/AC_CustomControl/Rover)

set(bin/ardurover_SOURCES
    AP_Arming_Rover.cpp
    AP_ExternalControl_Rover.cpp
    AP_Rally.cpp
    GCS_MAVLink_Rover.cpp
    GCS_Rover.cpp
    Log.cpp
    Parameters.cpp
    RC_Channel_Rover.cpp
    Rover.cpp
    Steering.cpp
    afs_rover.cpp
    balance_bot.cpp
    commands.cpp
    crash_check.cpp
    cruise_learn.cpp
    ekf_check.cpp
    failsafe.cpp
    fence.cpp
    mode.cpp
    mode_acro.cpp
    mode_auto.cpp
    mode_circle.cpp
    mode_dock.cpp
    mode_follow.cpp
    mode_guided.cpp
    mode_hold.cpp
    mode_loiter.cpp
    mode_manual.cpp
    mode_rtl.cpp
    mode_simple.cpp
    mode_smart_rtl.cpp
    mode_steering.cpp
    motor_test.cpp
    precision_landing.cpp
    radio.cpp
    sailboat.cpp
    sensors.cpp
    system.cpp)

set(bin/ardurover_INCLUDES
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

include_directories(${bin/ardurover_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover")

add_executable(bin/ardurover ${bin/ardurover_SOURCES})

target_link_libraries(bin/ardurover Rover_libs)

