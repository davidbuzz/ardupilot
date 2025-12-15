
set(objs/AP_AHRS/Blimp_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs/AP_AHRS/Blimp_INCLUDES
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

include_directories(${objs/AP_AHRS/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_AHRS/Blimp ${objs/AP_AHRS/Blimp_SOURCES})

target_link_libraries(objs/AP_AHRS/Blimp mavlink)
target_link_libraries(objs/AP_AHRS/Blimp dronecan)
target_link_libraries(objs/AP_AHRS/Blimp littlefs)

set(objs/AP_Airspeed/Blimp_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs/AP_Airspeed/Blimp_INCLUDES
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

include_directories(${objs/AP_Airspeed/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Airspeed/Blimp ${objs/AP_Airspeed/Blimp_SOURCES})

target_link_libraries(objs/AP_Airspeed/Blimp mavlink)
target_link_libraries(objs/AP_Airspeed/Blimp dronecan)
target_link_libraries(objs/AP_Airspeed/Blimp littlefs)

set(objs/AP_Baro/Blimp_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs/AP_Baro/Blimp_INCLUDES
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

include_directories(${objs/AP_Baro/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Baro/Blimp ${objs/AP_Baro/Blimp_SOURCES})

target_link_libraries(objs/AP_Baro/Blimp mavlink)
target_link_libraries(objs/AP_Baro/Blimp dronecan)
target_link_libraries(objs/AP_Baro/Blimp littlefs)

set(objs/AP_BattMonitor/Blimp_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs/AP_BattMonitor/Blimp_INCLUDES
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

include_directories(${objs/AP_BattMonitor/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_BattMonitor/Blimp ${objs/AP_BattMonitor/Blimp_SOURCES})

target_link_libraries(objs/AP_BattMonitor/Blimp mavlink)
target_link_libraries(objs/AP_BattMonitor/Blimp dronecan)
target_link_libraries(objs/AP_BattMonitor/Blimp littlefs)

set(objs/AP_BoardConfig/Blimp_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs/AP_BoardConfig/Blimp_INCLUDES
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

include_directories(${objs/AP_BoardConfig/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_BoardConfig/Blimp ${objs/AP_BoardConfig/Blimp_SOURCES})

target_link_libraries(objs/AP_BoardConfig/Blimp mavlink)
target_link_libraries(objs/AP_BoardConfig/Blimp dronecan)
target_link_libraries(objs/AP_BoardConfig/Blimp littlefs)

set(objs/AP_HAL/Blimp_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs/AP_HAL/Blimp_INCLUDES
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

include_directories(${objs/AP_HAL/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_HAL/Blimp ${objs/AP_HAL/Blimp_SOURCES})

target_link_libraries(objs/AP_HAL/Blimp mavlink)
target_link_libraries(objs/AP_HAL/Blimp dronecan)
target_link_libraries(objs/AP_HAL/Blimp littlefs)

set(objs/AP_InertialSensor/Blimp_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs/AP_InertialSensor/Blimp_INCLUDES
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

include_directories(${objs/AP_InertialSensor/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_InertialSensor/Blimp ${objs/AP_InertialSensor/Blimp_SOURCES})

target_link_libraries(objs/AP_InertialSensor/Blimp mavlink)
target_link_libraries(objs/AP_InertialSensor/Blimp dronecan)
target_link_libraries(objs/AP_InertialSensor/Blimp littlefs)

set(objs/AP_Math/Blimp_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs/AP_Math/Blimp_INCLUDES
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

include_directories(${objs/AP_Math/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Math/Blimp ${objs/AP_Math/Blimp_SOURCES})

target_link_libraries(objs/AP_Math/Blimp mavlink)
target_link_libraries(objs/AP_Math/Blimp dronecan)
target_link_libraries(objs/AP_Math/Blimp littlefs)

set(objs/AP_Mission/Blimp_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs/AP_Mission/Blimp_INCLUDES
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

include_directories(${objs/AP_Mission/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Mission/Blimp ${objs/AP_Mission/Blimp_SOURCES})

target_link_libraries(objs/AP_Mission/Blimp mavlink)
target_link_libraries(objs/AP_Mission/Blimp dronecan)
target_link_libraries(objs/AP_Mission/Blimp littlefs)

set(objs/AP_DAL/Blimp_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs/AP_DAL/Blimp_INCLUDES
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

include_directories(${objs/AP_DAL/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_DAL/Blimp ${objs/AP_DAL/Blimp_SOURCES})

target_link_libraries(objs/AP_DAL/Blimp mavlink)
target_link_libraries(objs/AP_DAL/Blimp dronecan)
target_link_libraries(objs/AP_DAL/Blimp littlefs)

set(objs/AP_NavEKF2/Blimp_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs/AP_NavEKF2/Blimp_INCLUDES
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

include_directories(${objs/AP_NavEKF2/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_NavEKF2/Blimp ${objs/AP_NavEKF2/Blimp_SOURCES})

target_link_libraries(objs/AP_NavEKF2/Blimp mavlink)
target_link_libraries(objs/AP_NavEKF2/Blimp dronecan)
target_link_libraries(objs/AP_NavEKF2/Blimp littlefs)

set(objs/AP_NavEKF3/Blimp_SOURCES
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

set(objs/AP_NavEKF3/Blimp_INCLUDES
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

include_directories(${objs/AP_NavEKF3/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_NavEKF3/Blimp ${objs/AP_NavEKF3/Blimp_SOURCES})

target_link_libraries(objs/AP_NavEKF3/Blimp mavlink)
target_link_libraries(objs/AP_NavEKF3/Blimp dronecan)
target_link_libraries(objs/AP_NavEKF3/Blimp littlefs)

set(objs/AP_Rally/Blimp_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs/AP_Rally/Blimp_INCLUDES
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

include_directories(${objs/AP_Rally/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Rally/Blimp ${objs/AP_Rally/Blimp_SOURCES})

target_link_libraries(objs/AP_Rally/Blimp mavlink)
target_link_libraries(objs/AP_Rally/Blimp dronecan)
target_link_libraries(objs/AP_Rally/Blimp littlefs)

set(objs/AP_Scheduler/Blimp_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs/AP_Scheduler/Blimp_INCLUDES
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

include_directories(${objs/AP_Scheduler/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Scheduler/Blimp ${objs/AP_Scheduler/Blimp_SOURCES})

target_link_libraries(objs/AP_Scheduler/Blimp mavlink)
target_link_libraries(objs/AP_Scheduler/Blimp dronecan)
target_link_libraries(objs/AP_Scheduler/Blimp littlefs)

set(objs/AP_Terrain/Blimp_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs/AP_Terrain/Blimp_INCLUDES
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

include_directories(${objs/AP_Terrain/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Terrain/Blimp ${objs/AP_Terrain/Blimp_SOURCES})

target_link_libraries(objs/AP_Terrain/Blimp mavlink)
target_link_libraries(objs/AP_Terrain/Blimp dronecan)
target_link_libraries(objs/AP_Terrain/Blimp littlefs)

set(objs/AP_Vehicle/Blimp_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs/AP_Vehicle/Blimp_INCLUDES
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

include_directories(${objs/AP_Vehicle/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Vehicle/Blimp ${objs/AP_Vehicle/Blimp_SOURCES})

target_link_libraries(objs/AP_Vehicle/Blimp mavlink)
target_link_libraries(objs/AP_Vehicle/Blimp dronecan)
target_link_libraries(objs/AP_Vehicle/Blimp littlefs)

set(objs/AP_Logger/Blimp_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs/AP_Logger/Blimp_INCLUDES
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

include_directories(${objs/AP_Logger/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Logger/Blimp ${objs/AP_Logger/Blimp_SOURCES})

target_link_libraries(objs/AP_Logger/Blimp mavlink)
target_link_libraries(objs/AP_Logger/Blimp dronecan)
target_link_libraries(objs/AP_Logger/Blimp littlefs)

set(objs/Filter/Blimp_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs/Filter/Blimp_INCLUDES
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

include_directories(${objs/Filter/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/Filter/Blimp ${objs/Filter/Blimp_SOURCES})

target_link_libraries(objs/Filter/Blimp mavlink)
target_link_libraries(objs/Filter/Blimp dronecan)
target_link_libraries(objs/Filter/Blimp littlefs)

set(objs/GCS_MAVLink/Blimp_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs/GCS_MAVLink/Blimp_INCLUDES
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

include_directories(${objs/GCS_MAVLink/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/GCS_MAVLink/Blimp ${objs/GCS_MAVLink/Blimp_SOURCES})

target_link_libraries(objs/GCS_MAVLink/Blimp mavlink)
target_link_libraries(objs/GCS_MAVLink/Blimp dronecan)
target_link_libraries(objs/GCS_MAVLink/Blimp littlefs)

set(objs/RC_Channel/Blimp_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs/RC_Channel/Blimp_INCLUDES
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

include_directories(${objs/RC_Channel/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/RC_Channel/Blimp ${objs/RC_Channel/Blimp_SOURCES})

target_link_libraries(objs/RC_Channel/Blimp mavlink)
target_link_libraries(objs/RC_Channel/Blimp dronecan)
target_link_libraries(objs/RC_Channel/Blimp littlefs)

set(objs/StorageManager/Blimp_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs/StorageManager/Blimp_INCLUDES
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

include_directories(${objs/StorageManager/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/StorageManager/Blimp ${objs/StorageManager/Blimp_SOURCES})

target_link_libraries(objs/StorageManager/Blimp mavlink)
target_link_libraries(objs/StorageManager/Blimp dronecan)
target_link_libraries(objs/StorageManager/Blimp littlefs)

set(objs/AP_Frsky_Telem/Blimp_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs/AP_Frsky_Telem/Blimp_INCLUDES
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

include_directories(${objs/AP_Frsky_Telem/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Frsky_Telem/Blimp ${objs/AP_Frsky_Telem/Blimp_SOURCES})

target_link_libraries(objs/AP_Frsky_Telem/Blimp mavlink)
target_link_libraries(objs/AP_Frsky_Telem/Blimp dronecan)
target_link_libraries(objs/AP_Frsky_Telem/Blimp littlefs)

set(objs/AP_IBus_Telem/Blimp_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs/AP_IBus_Telem/Blimp_INCLUDES
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

include_directories(${objs/AP_IBus_Telem/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_IBus_Telem/Blimp ${objs/AP_IBus_Telem/Blimp_SOURCES})

target_link_libraries(objs/AP_IBus_Telem/Blimp mavlink)
target_link_libraries(objs/AP_IBus_Telem/Blimp dronecan)
target_link_libraries(objs/AP_IBus_Telem/Blimp littlefs)

set(objs/AP_Relay/Blimp_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs/AP_Relay/Blimp_INCLUDES
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

include_directories(${objs/AP_Relay/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Relay/Blimp ${objs/AP_Relay/Blimp_SOURCES})

target_link_libraries(objs/AP_Relay/Blimp mavlink)
target_link_libraries(objs/AP_Relay/Blimp dronecan)
target_link_libraries(objs/AP_Relay/Blimp littlefs)

set(objs/AP_RCProtocol/Blimp_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs/AP_RCProtocol/Blimp_INCLUDES
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

include_directories(${objs/AP_RCProtocol/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_RCProtocol/Blimp ${objs/AP_RCProtocol/Blimp_SOURCES})

target_link_libraries(objs/AP_RCProtocol/Blimp mavlink)
target_link_libraries(objs/AP_RCProtocol/Blimp dronecan)
target_link_libraries(objs/AP_RCProtocol/Blimp littlefs)

set(objs/AP_BLHeli/Blimp_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs/AP_BLHeli/Blimp_INCLUDES
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

include_directories(${objs/AP_BLHeli/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_BLHeli/Blimp ${objs/AP_BLHeli/Blimp_SOURCES})

target_link_libraries(objs/AP_BLHeli/Blimp mavlink)
target_link_libraries(objs/AP_BLHeli/Blimp dronecan)
target_link_libraries(objs/AP_BLHeli/Blimp littlefs)

set(objs/AP_Proximity/Blimp_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs/AP_Proximity/Blimp_INCLUDES
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

include_directories(${objs/AP_Proximity/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Proximity/Blimp ${objs/AP_Proximity/Blimp_SOURCES})

target_link_libraries(objs/AP_Proximity/Blimp mavlink)
target_link_libraries(objs/AP_Proximity/Blimp dronecan)
target_link_libraries(objs/AP_Proximity/Blimp littlefs)

set(objs/AC_Fence/Blimp_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs/AC_Fence/Blimp_INCLUDES
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

include_directories(${objs/AC_Fence/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AC_Fence/Blimp ${objs/AC_Fence/Blimp_SOURCES})

target_link_libraries(objs/AC_Fence/Blimp mavlink)
target_link_libraries(objs/AC_Fence/Blimp dronecan)
target_link_libraries(objs/AC_Fence/Blimp littlefs)

set(objs/AC_Avoidance/Blimp_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs/AC_Avoidance/Blimp_INCLUDES
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

include_directories(${objs/AC_Avoidance/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AC_Avoidance/Blimp ${objs/AC_Avoidance/Blimp_SOURCES})

target_link_libraries(objs/AC_Avoidance/Blimp mavlink)
target_link_libraries(objs/AC_Avoidance/Blimp dronecan)
target_link_libraries(objs/AC_Avoidance/Blimp littlefs)

set(objs/AP_LandingGear/Blimp_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs/AP_LandingGear/Blimp_INCLUDES
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

include_directories(${objs/AP_LandingGear/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_LandingGear/Blimp ${objs/AP_LandingGear/Blimp_SOURCES})

target_link_libraries(objs/AP_LandingGear/Blimp mavlink)
target_link_libraries(objs/AP_LandingGear/Blimp dronecan)
target_link_libraries(objs/AP_LandingGear/Blimp littlefs)

set(objs/AP_OSD/Blimp_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs/AP_OSD/Blimp_INCLUDES
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

include_directories(${objs/AP_OSD/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_OSD/Blimp ${objs/AP_OSD/Blimp_SOURCES})

target_link_libraries(objs/AP_OSD/Blimp mavlink)
target_link_libraries(objs/AP_OSD/Blimp dronecan)
target_link_libraries(objs/AP_OSD/Blimp littlefs)

set(objs/AP_Filesystem/Blimp_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs/AP_Filesystem/Blimp_INCLUDES
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

include_directories(${objs/AP_Filesystem/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Filesystem/Blimp ${objs/AP_Filesystem/Blimp_SOURCES})

target_link_libraries(objs/AP_Filesystem/Blimp mavlink)
target_link_libraries(objs/AP_Filesystem/Blimp dronecan)
target_link_libraries(objs/AP_Filesystem/Blimp littlefs)

set(objs/AP_ADSB/Blimp_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs/AP_ADSB/Blimp_INCLUDES
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

include_directories(${objs/AP_ADSB/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_ADSB/Blimp ${objs/AP_ADSB/Blimp_SOURCES})

target_link_libraries(objs/AP_ADSB/Blimp mavlink)
target_link_libraries(objs/AP_ADSB/Blimp dronecan)
target_link_libraries(objs/AP_ADSB/Blimp littlefs)

set(objs/AP_ESC_Telem/Blimp_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs/AP_ESC_Telem/Blimp_INCLUDES
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

include_directories(${objs/AP_ESC_Telem/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_ESC_Telem/Blimp ${objs/AP_ESC_Telem/Blimp_SOURCES})

target_link_libraries(objs/AP_ESC_Telem/Blimp mavlink)
target_link_libraries(objs/AP_ESC_Telem/Blimp dronecan)
target_link_libraries(objs/AP_ESC_Telem/Blimp littlefs)

set(objs/AP_GyroFFT/Blimp_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs/AP_GyroFFT/Blimp_INCLUDES
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

include_directories(${objs/AP_GyroFFT/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_GyroFFT/Blimp ${objs/AP_GyroFFT/Blimp_SOURCES})

target_link_libraries(objs/AP_GyroFFT/Blimp mavlink)
target_link_libraries(objs/AP_GyroFFT/Blimp dronecan)
target_link_libraries(objs/AP_GyroFFT/Blimp littlefs)

set(objs/AP_RCTelemetry/Blimp_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs/AP_RCTelemetry/Blimp_INCLUDES
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

include_directories(${objs/AP_RCTelemetry/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_RCTelemetry/Blimp ${objs/AP_RCTelemetry/Blimp_SOURCES})

target_link_libraries(objs/AP_RCTelemetry/Blimp mavlink)
target_link_libraries(objs/AP_RCTelemetry/Blimp dronecan)
target_link_libraries(objs/AP_RCTelemetry/Blimp littlefs)

set(objs/AP_ExternalAHRS/Blimp_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs/AP_ExternalAHRS/Blimp_INCLUDES
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

include_directories(${objs/AP_ExternalAHRS/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_ExternalAHRS/Blimp ${objs/AP_ExternalAHRS/Blimp_SOURCES})

target_link_libraries(objs/AP_ExternalAHRS/Blimp mavlink)
target_link_libraries(objs/AP_ExternalAHRS/Blimp dronecan)
target_link_libraries(objs/AP_ExternalAHRS/Blimp littlefs)

set(objs/AP_TemperatureSensor/Blimp_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs/AP_TemperatureSensor/Blimp_INCLUDES
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

include_directories(${objs/AP_TemperatureSensor/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_TemperatureSensor/Blimp ${objs/AP_TemperatureSensor/Blimp_SOURCES})

target_link_libraries(objs/AP_TemperatureSensor/Blimp mavlink)
target_link_libraries(objs/AP_TemperatureSensor/Blimp dronecan)
target_link_libraries(objs/AP_TemperatureSensor/Blimp littlefs)

set(objs/AP_AIS/Blimp_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs/AP_AIS/Blimp_INCLUDES
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

include_directories(${objs/AP_AIS/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_AIS/Blimp ${objs/AP_AIS/Blimp_SOURCES})

target_link_libraries(objs/AP_AIS/Blimp mavlink)
target_link_libraries(objs/AP_AIS/Blimp dronecan)
target_link_libraries(objs/AP_AIS/Blimp littlefs)

set(objs/AP_Arming/Blimp_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs/AP_Arming/Blimp_INCLUDES
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

include_directories(${objs/AP_Arming/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Arming/Blimp ${objs/AP_Arming/Blimp_SOURCES})

target_link_libraries(objs/AP_Arming/Blimp mavlink)
target_link_libraries(objs/AP_Arming/Blimp dronecan)
target_link_libraries(objs/AP_Arming/Blimp littlefs)

set(objs/AP_Follow/Blimp_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs/AP_Follow/Blimp_INCLUDES
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

include_directories(${objs/AP_Follow/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Follow/Blimp ${objs/AP_Follow/Blimp_SOURCES})

target_link_libraries(objs/AP_Follow/Blimp mavlink)
target_link_libraries(objs/AP_Follow/Blimp dronecan)
target_link_libraries(objs/AP_Follow/Blimp littlefs)

set(objs/AP_Avoidance/Blimp_SOURCES
    ../libraries/AP_Avoidance/AP_Avoidance.cpp)

set(objs/AP_Avoidance/Blimp_INCLUDES
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

include_directories(${objs/AP_Avoidance/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Avoidance/Blimp ${objs/AP_Avoidance/Blimp_SOURCES})

target_link_libraries(objs/AP_Avoidance/Blimp mavlink)
target_link_libraries(objs/AP_Avoidance/Blimp dronecan)
target_link_libraries(objs/AP_Avoidance/Blimp littlefs)

set(objs/AC_AttitudeControl/Blimp_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs/AC_AttitudeControl/Blimp_INCLUDES
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

include_directories(${objs/AC_AttitudeControl/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AC_AttitudeControl/Blimp ${objs/AC_AttitudeControl/Blimp_SOURCES})

target_link_libraries(objs/AC_AttitudeControl/Blimp mavlink)
target_link_libraries(objs/AC_AttitudeControl/Blimp dronecan)
target_link_libraries(objs/AC_AttitudeControl/Blimp littlefs)

set(objs/AP_HAL_SITL/Blimp_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs/AP_HAL_SITL/Blimp_INCLUDES
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

include_directories(${objs/AP_HAL_SITL/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_HAL_SITL/Blimp ${objs/AP_HAL_SITL/Blimp_SOURCES})

target_link_libraries(objs/AP_HAL_SITL/Blimp mavlink)
target_link_libraries(objs/AP_HAL_SITL/Blimp dronecan)
target_link_libraries(objs/AP_HAL_SITL/Blimp littlefs)

set(objs/SITL/Blimp_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs/SITL/Blimp_INCLUDES
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

include_directories(${objs/SITL/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/SITL/Blimp ${objs/SITL/Blimp_SOURCES})

target_link_libraries(objs/SITL/Blimp mavlink)
target_link_libraries(objs/SITL/Blimp dronecan)
target_link_libraries(objs/SITL/Blimp littlefs)

set(objs/AP_Scripting/Blimp_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs/AP_Scripting/Blimp_INCLUDES
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

include_directories(${objs/AP_Scripting/Blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_library(objs/AP_Scripting/Blimp ${objs/AP_Scripting/Blimp_SOURCES})

target_link_libraries(objs/AP_Scripting/Blimp mavlink)
target_link_libraries(objs/AP_Scripting/Blimp dronecan)
target_link_libraries(objs/AP_Scripting/Blimp littlefs)

set(Blimp_libs_SOURCES
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

set(Blimp_libs_INCLUDES
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

include_directories(${Blimp_libs_INCLUDES})

add_library(Blimp_libs ${Blimp_libs_SOURCES})

target_link_libraries(Blimp_libs objs/AP_AccelCal)
target_link_libraries(Blimp_libs objs/AP_AccelCal/Blimp)
target_link_libraries(Blimp_libs objs/AP_ADC)
target_link_libraries(Blimp_libs objs/AP_ADC/Blimp)
target_link_libraries(Blimp_libs objs/AP_AHRS)
target_link_libraries(Blimp_libs objs/AP_AHRS/Blimp)
target_link_libraries(Blimp_libs objs/AP_Airspeed)
target_link_libraries(Blimp_libs objs/AP_Airspeed/Blimp)
target_link_libraries(Blimp_libs objs/AP_Baro)
target_link_libraries(Blimp_libs objs/AP_Baro/Blimp)
target_link_libraries(Blimp_libs objs/AP_BattMonitor)
target_link_libraries(Blimp_libs objs/AP_BattMonitor/Blimp)
target_link_libraries(Blimp_libs objs/AP_BoardConfig)
target_link_libraries(Blimp_libs objs/AP_BoardConfig/Blimp)
target_link_libraries(Blimp_libs objs/AP_Camera)
target_link_libraries(Blimp_libs objs/AP_Camera/Blimp)
target_link_libraries(Blimp_libs objs/AP_Common)
target_link_libraries(Blimp_libs objs/AP_Common/Blimp)
target_link_libraries(Blimp_libs objs/AP_Compass)
target_link_libraries(Blimp_libs objs/AP_Compass/Blimp)
target_link_libraries(Blimp_libs objs/AP_Declination)
target_link_libraries(Blimp_libs objs/AP_Declination/Blimp)
target_link_libraries(Blimp_libs objs/AP_GPS)
target_link_libraries(Blimp_libs objs/AP_GPS/Blimp)
target_link_libraries(Blimp_libs objs/AP_GSOF)
target_link_libraries(Blimp_libs objs/AP_GSOF/Blimp)
target_link_libraries(Blimp_libs objs/AP_HAL)
target_link_libraries(Blimp_libs objs/AP_HAL/Blimp)
target_link_libraries(Blimp_libs objs/AP_HAL_Empty)
target_link_libraries(Blimp_libs objs/AP_HAL_Empty/Blimp)
target_link_libraries(Blimp_libs objs/AP_DDS)
target_link_libraries(Blimp_libs objs/AP_DDS/Blimp)
target_link_libraries(Blimp_libs objs/AP_InertialSensor)
target_link_libraries(Blimp_libs objs/AP_InertialSensor/Blimp)
target_link_libraries(Blimp_libs objs/AP_Math)
target_link_libraries(Blimp_libs objs/AP_Math/Blimp)
target_link_libraries(Blimp_libs objs/AP_Mission)
target_link_libraries(Blimp_libs objs/AP_Mission/Blimp)
target_link_libraries(Blimp_libs objs/AP_DAL)
target_link_libraries(Blimp_libs objs/AP_DAL/Blimp)
target_link_libraries(Blimp_libs objs/AP_NavEKF)
target_link_libraries(Blimp_libs objs/AP_NavEKF/Blimp)
target_link_libraries(Blimp_libs objs/AP_NavEKF2)
target_link_libraries(Blimp_libs objs/AP_NavEKF2/Blimp)
target_link_libraries(Blimp_libs objs/AP_NavEKF3)
target_link_libraries(Blimp_libs objs/AP_NavEKF3/Blimp)
target_link_libraries(Blimp_libs objs/AP_Notify)
target_link_libraries(Blimp_libs objs/AP_Notify/Blimp)
target_link_libraries(Blimp_libs objs/AP_OpticalFlow)
target_link_libraries(Blimp_libs objs/AP_OpticalFlow/Blimp)
target_link_libraries(Blimp_libs objs/AP_Param)
target_link_libraries(Blimp_libs objs/AP_Param/Blimp)
target_link_libraries(Blimp_libs objs/AP_Rally)
target_link_libraries(Blimp_libs objs/AP_Rally/Blimp)
target_link_libraries(Blimp_libs objs/AP_LightWareSerial)
target_link_libraries(Blimp_libs objs/AP_LightWareSerial/Blimp)
target_link_libraries(Blimp_libs objs/AP_RangeFinder)
target_link_libraries(Blimp_libs objs/AP_RangeFinder/Blimp)
target_link_libraries(Blimp_libs objs/AP_Scheduler)
target_link_libraries(Blimp_libs objs/AP_Scheduler/Blimp)
target_link_libraries(Blimp_libs objs/AP_SerialManager)
target_link_libraries(Blimp_libs objs/AP_SerialManager/Blimp)
target_link_libraries(Blimp_libs objs/AP_Terrain)
target_link_libraries(Blimp_libs objs/AP_Terrain/Blimp)
target_link_libraries(Blimp_libs objs/AP_Vehicle)
target_link_libraries(Blimp_libs objs/AP_Vehicle/Blimp)
target_link_libraries(Blimp_libs objs/AP_InternalError)
target_link_libraries(Blimp_libs objs/AP_InternalError/Blimp)
target_link_libraries(Blimp_libs objs/AP_Logger)
target_link_libraries(Blimp_libs objs/AP_Logger/Blimp)
target_link_libraries(Blimp_libs objs/Filter)
target_link_libraries(Blimp_libs objs/Filter/Blimp)
target_link_libraries(Blimp_libs objs/GCS_MAVLink)
target_link_libraries(Blimp_libs objs/GCS_MAVLink/Blimp)
target_link_libraries(Blimp_libs objs/RC_Channel)
target_link_libraries(Blimp_libs objs/RC_Channel/Blimp)
target_link_libraries(Blimp_libs objs/SRV_Channel)
target_link_libraries(Blimp_libs objs/SRV_Channel/Blimp)
target_link_libraries(Blimp_libs objs/StorageManager)
target_link_libraries(Blimp_libs objs/StorageManager/Blimp)
target_link_libraries(Blimp_libs objs/AP_Tuning)
target_link_libraries(Blimp_libs objs/AP_Tuning/Blimp)
target_link_libraries(Blimp_libs objs/AP_RPM)
target_link_libraries(Blimp_libs objs/AP_RPM/Blimp)
target_link_libraries(Blimp_libs objs/AP_RSSI)
target_link_libraries(Blimp_libs objs/AP_RSSI/Blimp)
target_link_libraries(Blimp_libs objs/AP_Mount)
target_link_libraries(Blimp_libs objs/AP_Mount/Blimp)
target_link_libraries(Blimp_libs objs/AP_Module)
target_link_libraries(Blimp_libs objs/AP_Module/Blimp)
target_link_libraries(Blimp_libs objs/AP_Button)
target_link_libraries(Blimp_libs objs/AP_Button/Blimp)
target_link_libraries(Blimp_libs objs/AP_ICEngine)
target_link_libraries(Blimp_libs objs/AP_ICEngine/Blimp)
target_link_libraries(Blimp_libs objs/AP_Networking)
target_link_libraries(Blimp_libs objs/AP_Networking/Blimp)
target_link_libraries(Blimp_libs objs/AP_Frsky_Telem)
target_link_libraries(Blimp_libs objs/AP_Frsky_Telem/Blimp)
target_link_libraries(Blimp_libs objs/AP_IBus_Telem)
target_link_libraries(Blimp_libs objs/AP_IBus_Telem/Blimp)
target_link_libraries(Blimp_libs objs/AP_FlashStorage)
target_link_libraries(Blimp_libs objs/AP_FlashStorage/Blimp)
target_link_libraries(Blimp_libs objs/AP_Relay)
target_link_libraries(Blimp_libs objs/AP_Relay/Blimp)
target_link_libraries(Blimp_libs objs/AP_ServoRelayEvents)
target_link_libraries(Blimp_libs objs/AP_ServoRelayEvents/Blimp)
target_link_libraries(Blimp_libs objs/AP_Volz_Protocol)
target_link_libraries(Blimp_libs objs/AP_Volz_Protocol/Blimp)
target_link_libraries(Blimp_libs objs/AP_SBusOut)
target_link_libraries(Blimp_libs objs/AP_SBusOut/Blimp)
target_link_libraries(Blimp_libs objs/AP_IOMCU)
target_link_libraries(Blimp_libs objs/AP_IOMCU/Blimp)
target_link_libraries(Blimp_libs objs/AP_Parachute)
target_link_libraries(Blimp_libs objs/AP_Parachute/Blimp)
target_link_libraries(Blimp_libs objs/AP_RAMTRON)
target_link_libraries(Blimp_libs objs/AP_RAMTRON/Blimp)
target_link_libraries(Blimp_libs objs/AP_RCProtocol)
target_link_libraries(Blimp_libs objs/AP_RCProtocol/Blimp)
target_link_libraries(Blimp_libs objs/AP_Radio)
target_link_libraries(Blimp_libs objs/AP_Radio/Blimp)
target_link_libraries(Blimp_libs objs/AP_TempCalibration)
target_link_libraries(Blimp_libs objs/AP_TempCalibration/Blimp)
target_link_libraries(Blimp_libs objs/AP_VisualOdom)
target_link_libraries(Blimp_libs objs/AP_VisualOdom/Blimp)
target_link_libraries(Blimp_libs objs/AP_BLHeli)
target_link_libraries(Blimp_libs objs/AP_BLHeli/Blimp)
target_link_libraries(Blimp_libs objs/AP_ROMFS)
target_link_libraries(Blimp_libs objs/AP_ROMFS/Blimp)
target_link_libraries(Blimp_libs objs/AP_Proximity)
target_link_libraries(Blimp_libs objs/AP_Proximity/Blimp)
target_link_libraries(Blimp_libs objs/AP_Gripper)
target_link_libraries(Blimp_libs objs/AP_Gripper/Blimp)
target_link_libraries(Blimp_libs objs/AP_RTC)
target_link_libraries(Blimp_libs objs/AP_RTC/Blimp)
target_link_libraries(Blimp_libs objs/AC_Sprayer)
target_link_libraries(Blimp_libs objs/AC_Sprayer/Blimp)
target_link_libraries(Blimp_libs objs/AC_Fence)
target_link_libraries(Blimp_libs objs/AC_Fence/Blimp)
target_link_libraries(Blimp_libs objs/AC_Avoidance)
target_link_libraries(Blimp_libs objs/AC_Avoidance/Blimp)
target_link_libraries(Blimp_libs objs/AP_LandingGear)
target_link_libraries(Blimp_libs objs/AP_LandingGear/Blimp)
target_link_libraries(Blimp_libs objs/AP_RobotisServo)
target_link_libraries(Blimp_libs objs/AP_RobotisServo/Blimp)
target_link_libraries(Blimp_libs objs/AP_NMEA_Output)
target_link_libraries(Blimp_libs objs/AP_NMEA_Output/Blimp)
target_link_libraries(Blimp_libs objs/AP_OSD)
target_link_libraries(Blimp_libs objs/AP_OSD/Blimp)
target_link_libraries(Blimp_libs objs/AP_Filesystem)
target_link_libraries(Blimp_libs objs/AP_Filesystem/Blimp)
target_link_libraries(Blimp_libs objs/AP_ADSB)
target_link_libraries(Blimp_libs objs/AP_ADSB/Blimp)
target_link_libraries(Blimp_libs objs/AP_ADSB/sagetech-sdk)
target_link_libraries(Blimp_libs objs/AP_ADSB/sagetech-sdk/Blimp)
target_link_libraries(Blimp_libs objs/AC_PID)
target_link_libraries(Blimp_libs objs/AC_PID/Blimp)
target_link_libraries(Blimp_libs objs/AP_SerialLED)
target_link_libraries(Blimp_libs objs/AP_SerialLED/Blimp)
target_link_libraries(Blimp_libs objs/AP_EFI)
target_link_libraries(Blimp_libs objs/AP_EFI/Blimp)
target_link_libraries(Blimp_libs objs/AP_Hott_Telem)
target_link_libraries(Blimp_libs objs/AP_Hott_Telem/Blimp)
target_link_libraries(Blimp_libs objs/AP_ESC_Telem)
target_link_libraries(Blimp_libs objs/AP_ESC_Telem/Blimp)
target_link_libraries(Blimp_libs objs/AP_Servo_Telem)
target_link_libraries(Blimp_libs objs/AP_Servo_Telem/Blimp)
target_link_libraries(Blimp_libs objs/AP_Stats)
target_link_libraries(Blimp_libs objs/AP_Stats/Blimp)
target_link_libraries(Blimp_libs objs/AP_GyroFFT)
target_link_libraries(Blimp_libs objs/AP_GyroFFT/Blimp)
target_link_libraries(Blimp_libs objs/AP_RCTelemetry)
target_link_libraries(Blimp_libs objs/AP_RCTelemetry/Blimp)
target_link_libraries(Blimp_libs objs/AP_Generator)
target_link_libraries(Blimp_libs objs/AP_Generator/Blimp)
target_link_libraries(Blimp_libs objs/AP_MSP)
target_link_libraries(Blimp_libs objs/AP_MSP/Blimp)
target_link_libraries(Blimp_libs objs/AP_OLC)
target_link_libraries(Blimp_libs objs/AP_OLC/Blimp)
target_link_libraries(Blimp_libs objs/AP_WheelEncoder)
target_link_libraries(Blimp_libs objs/AP_WheelEncoder/Blimp)
target_link_libraries(Blimp_libs objs/AP_ExternalAHRS)
target_link_libraries(Blimp_libs objs/AP_ExternalAHRS/Blimp)
target_link_libraries(Blimp_libs objs/AP_VideoTX)
target_link_libraries(Blimp_libs objs/AP_VideoTX/Blimp)
target_link_libraries(Blimp_libs objs/AP_FETtecOneWire)
target_link_libraries(Blimp_libs objs/AP_FETtecOneWire/Blimp)
target_link_libraries(Blimp_libs objs/AP_TemperatureSensor)
target_link_libraries(Blimp_libs objs/AP_TemperatureSensor/Blimp)
target_link_libraries(Blimp_libs objs/AP_Torqeedo)
target_link_libraries(Blimp_libs objs/AP_Torqeedo/Blimp)
target_link_libraries(Blimp_libs objs/AP_CustomRotations)
target_link_libraries(Blimp_libs objs/AP_CustomRotations/Blimp)
target_link_libraries(Blimp_libs objs/AP_AIS)
target_link_libraries(Blimp_libs objs/AP_AIS/Blimp)
target_link_libraries(Blimp_libs objs/AP_OpenDroneID)
target_link_libraries(Blimp_libs objs/AP_OpenDroneID/Blimp)
target_link_libraries(Blimp_libs objs/AP_CheckFirmware)
target_link_libraries(Blimp_libs objs/AP_CheckFirmware/Blimp)
target_link_libraries(Blimp_libs objs/AP_ExternalControl)
target_link_libraries(Blimp_libs objs/AP_ExternalControl/Blimp)
target_link_libraries(Blimp_libs objs/AP_JSON)
target_link_libraries(Blimp_libs objs/AP_JSON/Blimp)
target_link_libraries(Blimp_libs objs/AP_Beacon)
target_link_libraries(Blimp_libs objs/AP_Beacon/Blimp)
target_link_libraries(Blimp_libs objs/AP_Arming)
target_link_libraries(Blimp_libs objs/AP_Arming/Blimp)
target_link_libraries(Blimp_libs objs/AP_RCMapper)
target_link_libraries(Blimp_libs objs/AP_RCMapper/Blimp)
target_link_libraries(Blimp_libs objs/AP_MultiHeap)
target_link_libraries(Blimp_libs objs/AP_MultiHeap/Blimp)
target_link_libraries(Blimp_libs objs/AP_Follow)
target_link_libraries(Blimp_libs objs/AP_Follow/Blimp)
target_link_libraries(Blimp_libs objs/AP_CANManager)
target_link_libraries(Blimp_libs objs/AP_CANManager/Blimp)
target_link_libraries(Blimp_libs objs/AP_KDECAN)
target_link_libraries(Blimp_libs objs/AP_KDECAN/Blimp)
target_link_libraries(Blimp_libs objs/AP_PiccoloCAN)
target_link_libraries(Blimp_libs objs/AP_PiccoloCAN/Blimp)
target_link_libraries(Blimp_libs objs/AP_PiccoloCAN/piccolo_protocol)
target_link_libraries(Blimp_libs objs/AP_PiccoloCAN/piccolo_protocol/Blimp)
target_link_libraries(Blimp_libs objs/AC_InputManager)
target_link_libraries(Blimp_libs objs/AC_InputManager/Blimp)
target_link_libraries(Blimp_libs objs/AP_InertialNav)
target_link_libraries(Blimp_libs objs/AP_InertialNav/Blimp)
target_link_libraries(Blimp_libs objs/AP_Avoidance)
target_link_libraries(Blimp_libs objs/AP_Avoidance/Blimp)
target_link_libraries(Blimp_libs objs/AP_LTM_Telem)
target_link_libraries(Blimp_libs objs/AP_LTM_Telem/Blimp)
target_link_libraries(Blimp_libs objs/AP_Devo_Telem)
target_link_libraries(Blimp_libs objs/AP_Devo_Telem/Blimp)
target_link_libraries(Blimp_libs objs/AP_AdvancedFailsafe)
target_link_libraries(Blimp_libs objs/AP_AdvancedFailsafe/Blimp)
target_link_libraries(Blimp_libs objs/AC_AttitudeControl)
target_link_libraries(Blimp_libs objs/AC_AttitudeControl/Blimp)
target_link_libraries(Blimp_libs objs/AP_DroneCAN)
target_link_libraries(Blimp_libs objs/AP_DroneCAN/Blimp)
target_link_libraries(Blimp_libs objs/modules/DroneCAN/libcanard/*.c)
target_link_libraries(Blimp_libs objs/modules/DroneCAN/libcanard/*.c/Blimp)
target_link_libraries(Blimp_libs objs/AP_HAL_SITL)
target_link_libraries(Blimp_libs objs/AP_HAL_SITL/Blimp)
target_link_libraries(Blimp_libs objs/AP_CSVReader)
target_link_libraries(Blimp_libs objs/AP_CSVReader/Blimp)
target_link_libraries(Blimp_libs objs/SITL)
target_link_libraries(Blimp_libs objs/SITL/Blimp)
target_link_libraries(Blimp_libs objs/AP_Scripting)
target_link_libraries(Blimp_libs objs/AP_Scripting/Blimp)
target_link_libraries(Blimp_libs objs/AP_Scripting/lua/src)
target_link_libraries(Blimp_libs objs/AP_Scripting/lua/src/Blimp)
target_link_libraries(Blimp_libs objs/AC_CustomControl)
target_link_libraries(Blimp_libs objs/AC_CustomControl/Blimp)

set(bin/blimp_SOURCES
    AP_Arming_Blimp.cpp
    AP_State.cpp
    Blimp.cpp
    Fins.cpp
    GCS_Blimp.cpp
    GCS_MAVLink_Blimp.cpp
    Log.cpp
    Loiter.cpp
    Parameters.cpp
    RC_Channel_Blimp.cpp
    commands.cpp
    ekf_check.cpp
    events.cpp
    failsafe.cpp
    inertia.cpp
    mode.cpp
    mode_land.cpp
    mode_loiter.cpp
    mode_manual.cpp
    mode_rtl.cpp
    mode_velocity.cpp
    motors.cpp
    radio.cpp
    sensors.cpp
    system.cpp)

set(bin/blimp_INCLUDES
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

include_directories(${bin/blimp_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp")

add_executable(bin/blimp ${bin/blimp_SOURCES})

target_link_libraries(bin/blimp B)
target_link_libraries(bin/blimp l)
target_link_libraries(bin/blimp i)
target_link_libraries(bin/blimp m)
target_link_libraries(bin/blimp p)
target_link_libraries(bin/blimp _)
target_link_libraries(bin/blimp l)
target_link_libraries(bin/blimp i)
target_link_libraries(bin/blimp b)
target_link_libraries(bin/blimp s)

