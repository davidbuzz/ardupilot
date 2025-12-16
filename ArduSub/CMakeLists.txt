
set(objs/AP_AHRS/ArduSub_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs/AP_AHRS/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_AHRS/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_AHRS/ArduSub ${objs/AP_AHRS/ArduSub_SOURCES})

target_link_libraries(objs/AP_AHRS/ArduSub mavlink)
target_link_libraries(objs/AP_AHRS/ArduSub dronecan)
target_link_libraries(objs/AP_AHRS/ArduSub littlefs)

set(objs/AP_Airspeed/ArduSub_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs/AP_Airspeed/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Airspeed/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Airspeed/ArduSub ${objs/AP_Airspeed/ArduSub_SOURCES})

target_link_libraries(objs/AP_Airspeed/ArduSub mavlink)
target_link_libraries(objs/AP_Airspeed/ArduSub dronecan)
target_link_libraries(objs/AP_Airspeed/ArduSub littlefs)

set(objs/AP_Baro/ArduSub_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs/AP_Baro/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Baro/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Baro/ArduSub ${objs/AP_Baro/ArduSub_SOURCES})

target_link_libraries(objs/AP_Baro/ArduSub mavlink)
target_link_libraries(objs/AP_Baro/ArduSub dronecan)
target_link_libraries(objs/AP_Baro/ArduSub littlefs)

set(objs/AP_BattMonitor/ArduSub_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs/AP_BattMonitor/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_BattMonitor/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_BattMonitor/ArduSub ${objs/AP_BattMonitor/ArduSub_SOURCES})

target_link_libraries(objs/AP_BattMonitor/ArduSub mavlink)
target_link_libraries(objs/AP_BattMonitor/ArduSub dronecan)
target_link_libraries(objs/AP_BattMonitor/ArduSub littlefs)

set(objs/AP_BoardConfig/ArduSub_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs/AP_BoardConfig/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_BoardConfig/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_BoardConfig/ArduSub ${objs/AP_BoardConfig/ArduSub_SOURCES})

target_link_libraries(objs/AP_BoardConfig/ArduSub mavlink)
target_link_libraries(objs/AP_BoardConfig/ArduSub dronecan)
target_link_libraries(objs/AP_BoardConfig/ArduSub littlefs)

set(objs/AP_HAL/ArduSub_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs/AP_HAL/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_HAL/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_HAL/ArduSub ${objs/AP_HAL/ArduSub_SOURCES})

target_link_libraries(objs/AP_HAL/ArduSub mavlink)
target_link_libraries(objs/AP_HAL/ArduSub dronecan)
target_link_libraries(objs/AP_HAL/ArduSub littlefs)

set(objs/AP_InertialSensor/ArduSub_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs/AP_InertialSensor/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_InertialSensor/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_InertialSensor/ArduSub ${objs/AP_InertialSensor/ArduSub_SOURCES})

target_link_libraries(objs/AP_InertialSensor/ArduSub mavlink)
target_link_libraries(objs/AP_InertialSensor/ArduSub dronecan)
target_link_libraries(objs/AP_InertialSensor/ArduSub littlefs)

set(objs/AP_Math/ArduSub_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs/AP_Math/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Math/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Math/ArduSub ${objs/AP_Math/ArduSub_SOURCES})

target_link_libraries(objs/AP_Math/ArduSub mavlink)
target_link_libraries(objs/AP_Math/ArduSub dronecan)
target_link_libraries(objs/AP_Math/ArduSub littlefs)

set(objs/AP_Mission/ArduSub_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs/AP_Mission/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Mission/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Mission/ArduSub ${objs/AP_Mission/ArduSub_SOURCES})

target_link_libraries(objs/AP_Mission/ArduSub mavlink)
target_link_libraries(objs/AP_Mission/ArduSub dronecan)
target_link_libraries(objs/AP_Mission/ArduSub littlefs)

set(objs/AP_DAL/ArduSub_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs/AP_DAL/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_DAL/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_DAL/ArduSub ${objs/AP_DAL/ArduSub_SOURCES})

target_link_libraries(objs/AP_DAL/ArduSub mavlink)
target_link_libraries(objs/AP_DAL/ArduSub dronecan)
target_link_libraries(objs/AP_DAL/ArduSub littlefs)

set(objs/AP_NavEKF2/ArduSub_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs/AP_NavEKF2/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_NavEKF2/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_NavEKF2/ArduSub ${objs/AP_NavEKF2/ArduSub_SOURCES})

target_link_libraries(objs/AP_NavEKF2/ArduSub mavlink)
target_link_libraries(objs/AP_NavEKF2/ArduSub dronecan)
target_link_libraries(objs/AP_NavEKF2/ArduSub littlefs)

set(objs/AP_NavEKF3/ArduSub_SOURCES
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

set(objs/AP_NavEKF3/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_NavEKF3/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_NavEKF3/ArduSub ${objs/AP_NavEKF3/ArduSub_SOURCES})

target_link_libraries(objs/AP_NavEKF3/ArduSub mavlink)
target_link_libraries(objs/AP_NavEKF3/ArduSub dronecan)
target_link_libraries(objs/AP_NavEKF3/ArduSub littlefs)

set(objs/AP_Rally/ArduSub_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs/AP_Rally/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Rally/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Rally/ArduSub ${objs/AP_Rally/ArduSub_SOURCES})

target_link_libraries(objs/AP_Rally/ArduSub mavlink)
target_link_libraries(objs/AP_Rally/ArduSub dronecan)
target_link_libraries(objs/AP_Rally/ArduSub littlefs)

set(objs/AP_Scheduler/ArduSub_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs/AP_Scheduler/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Scheduler/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Scheduler/ArduSub ${objs/AP_Scheduler/ArduSub_SOURCES})

target_link_libraries(objs/AP_Scheduler/ArduSub mavlink)
target_link_libraries(objs/AP_Scheduler/ArduSub dronecan)
target_link_libraries(objs/AP_Scheduler/ArduSub littlefs)

set(objs/AP_Terrain/ArduSub_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs/AP_Terrain/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Terrain/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Terrain/ArduSub ${objs/AP_Terrain/ArduSub_SOURCES})

target_link_libraries(objs/AP_Terrain/ArduSub mavlink)
target_link_libraries(objs/AP_Terrain/ArduSub dronecan)
target_link_libraries(objs/AP_Terrain/ArduSub littlefs)

set(objs/AP_Vehicle/ArduSub_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs/AP_Vehicle/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Vehicle/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Vehicle/ArduSub ${objs/AP_Vehicle/ArduSub_SOURCES})

target_link_libraries(objs/AP_Vehicle/ArduSub mavlink)
target_link_libraries(objs/AP_Vehicle/ArduSub dronecan)
target_link_libraries(objs/AP_Vehicle/ArduSub littlefs)

set(objs/AP_Logger/ArduSub_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs/AP_Logger/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Logger/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Logger/ArduSub ${objs/AP_Logger/ArduSub_SOURCES})

target_link_libraries(objs/AP_Logger/ArduSub mavlink)
target_link_libraries(objs/AP_Logger/ArduSub dronecan)
target_link_libraries(objs/AP_Logger/ArduSub littlefs)

set(objs/Filter/ArduSub_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs/Filter/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/Filter/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/Filter/ArduSub ${objs/Filter/ArduSub_SOURCES})

target_link_libraries(objs/Filter/ArduSub mavlink)
target_link_libraries(objs/Filter/ArduSub dronecan)
target_link_libraries(objs/Filter/ArduSub littlefs)

set(objs/GCS_MAVLink/ArduSub_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs/GCS_MAVLink/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/GCS_MAVLink/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/GCS_MAVLink/ArduSub ${objs/GCS_MAVLink/ArduSub_SOURCES})

target_link_libraries(objs/GCS_MAVLink/ArduSub mavlink)
target_link_libraries(objs/GCS_MAVLink/ArduSub dronecan)
target_link_libraries(objs/GCS_MAVLink/ArduSub littlefs)

set(objs/RC_Channel/ArduSub_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs/RC_Channel/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/RC_Channel/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/RC_Channel/ArduSub ${objs/RC_Channel/ArduSub_SOURCES})

target_link_libraries(objs/RC_Channel/ArduSub mavlink)
target_link_libraries(objs/RC_Channel/ArduSub dronecan)
target_link_libraries(objs/RC_Channel/ArduSub littlefs)

set(objs/StorageManager/ArduSub_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs/StorageManager/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/StorageManager/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/StorageManager/ArduSub ${objs/StorageManager/ArduSub_SOURCES})

target_link_libraries(objs/StorageManager/ArduSub mavlink)
target_link_libraries(objs/StorageManager/ArduSub dronecan)
target_link_libraries(objs/StorageManager/ArduSub littlefs)

set(objs/AP_Frsky_Telem/ArduSub_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs/AP_Frsky_Telem/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Frsky_Telem/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Frsky_Telem/ArduSub ${objs/AP_Frsky_Telem/ArduSub_SOURCES})

target_link_libraries(objs/AP_Frsky_Telem/ArduSub mavlink)
target_link_libraries(objs/AP_Frsky_Telem/ArduSub dronecan)
target_link_libraries(objs/AP_Frsky_Telem/ArduSub littlefs)

set(objs/AP_IBus_Telem/ArduSub_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs/AP_IBus_Telem/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_IBus_Telem/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_IBus_Telem/ArduSub ${objs/AP_IBus_Telem/ArduSub_SOURCES})

target_link_libraries(objs/AP_IBus_Telem/ArduSub mavlink)
target_link_libraries(objs/AP_IBus_Telem/ArduSub dronecan)
target_link_libraries(objs/AP_IBus_Telem/ArduSub littlefs)

set(objs/AP_Relay/ArduSub_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs/AP_Relay/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Relay/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Relay/ArduSub ${objs/AP_Relay/ArduSub_SOURCES})

target_link_libraries(objs/AP_Relay/ArduSub mavlink)
target_link_libraries(objs/AP_Relay/ArduSub dronecan)
target_link_libraries(objs/AP_Relay/ArduSub littlefs)

set(objs/AP_RCProtocol/ArduSub_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs/AP_RCProtocol/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_RCProtocol/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_RCProtocol/ArduSub ${objs/AP_RCProtocol/ArduSub_SOURCES})

target_link_libraries(objs/AP_RCProtocol/ArduSub mavlink)
target_link_libraries(objs/AP_RCProtocol/ArduSub dronecan)
target_link_libraries(objs/AP_RCProtocol/ArduSub littlefs)

set(objs/AP_BLHeli/ArduSub_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs/AP_BLHeli/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_BLHeli/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_BLHeli/ArduSub ${objs/AP_BLHeli/ArduSub_SOURCES})

target_link_libraries(objs/AP_BLHeli/ArduSub mavlink)
target_link_libraries(objs/AP_BLHeli/ArduSub dronecan)
target_link_libraries(objs/AP_BLHeli/ArduSub littlefs)

set(objs/AP_Proximity/ArduSub_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs/AP_Proximity/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Proximity/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Proximity/ArduSub ${objs/AP_Proximity/ArduSub_SOURCES})

target_link_libraries(objs/AP_Proximity/ArduSub mavlink)
target_link_libraries(objs/AP_Proximity/ArduSub dronecan)
target_link_libraries(objs/AP_Proximity/ArduSub littlefs)

set(objs/AC_Fence/ArduSub_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs/AC_Fence/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_Fence/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AC_Fence/ArduSub ${objs/AC_Fence/ArduSub_SOURCES})

target_link_libraries(objs/AC_Fence/ArduSub mavlink)
target_link_libraries(objs/AC_Fence/ArduSub dronecan)
target_link_libraries(objs/AC_Fence/ArduSub littlefs)

set(objs/AC_Avoidance/ArduSub_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs/AC_Avoidance/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_Avoidance/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AC_Avoidance/ArduSub ${objs/AC_Avoidance/ArduSub_SOURCES})

target_link_libraries(objs/AC_Avoidance/ArduSub mavlink)
target_link_libraries(objs/AC_Avoidance/ArduSub dronecan)
target_link_libraries(objs/AC_Avoidance/ArduSub littlefs)

set(objs/AP_LandingGear/ArduSub_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs/AP_LandingGear/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_LandingGear/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_LandingGear/ArduSub ${objs/AP_LandingGear/ArduSub_SOURCES})

target_link_libraries(objs/AP_LandingGear/ArduSub mavlink)
target_link_libraries(objs/AP_LandingGear/ArduSub dronecan)
target_link_libraries(objs/AP_LandingGear/ArduSub littlefs)

set(objs/AP_OSD/ArduSub_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs/AP_OSD/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_OSD/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_OSD/ArduSub ${objs/AP_OSD/ArduSub_SOURCES})

target_link_libraries(objs/AP_OSD/ArduSub mavlink)
target_link_libraries(objs/AP_OSD/ArduSub dronecan)
target_link_libraries(objs/AP_OSD/ArduSub littlefs)

set(objs/AP_Filesystem/ArduSub_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs/AP_Filesystem/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Filesystem/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Filesystem/ArduSub ${objs/AP_Filesystem/ArduSub_SOURCES})

target_link_libraries(objs/AP_Filesystem/ArduSub mavlink)
target_link_libraries(objs/AP_Filesystem/ArduSub dronecan)
target_link_libraries(objs/AP_Filesystem/ArduSub littlefs)

set(objs/AP_ADSB/ArduSub_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs/AP_ADSB/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_ADSB/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_ADSB/ArduSub ${objs/AP_ADSB/ArduSub_SOURCES})

target_link_libraries(objs/AP_ADSB/ArduSub mavlink)
target_link_libraries(objs/AP_ADSB/ArduSub dronecan)
target_link_libraries(objs/AP_ADSB/ArduSub littlefs)

set(objs/AP_ESC_Telem/ArduSub_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs/AP_ESC_Telem/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_ESC_Telem/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_ESC_Telem/ArduSub ${objs/AP_ESC_Telem/ArduSub_SOURCES})

target_link_libraries(objs/AP_ESC_Telem/ArduSub mavlink)
target_link_libraries(objs/AP_ESC_Telem/ArduSub dronecan)
target_link_libraries(objs/AP_ESC_Telem/ArduSub littlefs)

set(objs/AP_GyroFFT/ArduSub_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs/AP_GyroFFT/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_GyroFFT/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_GyroFFT/ArduSub ${objs/AP_GyroFFT/ArduSub_SOURCES})

target_link_libraries(objs/AP_GyroFFT/ArduSub mavlink)
target_link_libraries(objs/AP_GyroFFT/ArduSub dronecan)
target_link_libraries(objs/AP_GyroFFT/ArduSub littlefs)

set(objs/AP_RCTelemetry/ArduSub_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs/AP_RCTelemetry/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_RCTelemetry/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_RCTelemetry/ArduSub ${objs/AP_RCTelemetry/ArduSub_SOURCES})

target_link_libraries(objs/AP_RCTelemetry/ArduSub mavlink)
target_link_libraries(objs/AP_RCTelemetry/ArduSub dronecan)
target_link_libraries(objs/AP_RCTelemetry/ArduSub littlefs)

set(objs/AP_ExternalAHRS/ArduSub_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs/AP_ExternalAHRS/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_ExternalAHRS/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_ExternalAHRS/ArduSub ${objs/AP_ExternalAHRS/ArduSub_SOURCES})

target_link_libraries(objs/AP_ExternalAHRS/ArduSub mavlink)
target_link_libraries(objs/AP_ExternalAHRS/ArduSub dronecan)
target_link_libraries(objs/AP_ExternalAHRS/ArduSub littlefs)

set(objs/AP_TemperatureSensor/ArduSub_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs/AP_TemperatureSensor/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_TemperatureSensor/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_TemperatureSensor/ArduSub ${objs/AP_TemperatureSensor/ArduSub_SOURCES})

target_link_libraries(objs/AP_TemperatureSensor/ArduSub mavlink)
target_link_libraries(objs/AP_TemperatureSensor/ArduSub dronecan)
target_link_libraries(objs/AP_TemperatureSensor/ArduSub littlefs)

set(objs/AP_AIS/ArduSub_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs/AP_AIS/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_AIS/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_AIS/ArduSub ${objs/AP_AIS/ArduSub_SOURCES})

target_link_libraries(objs/AP_AIS/ArduSub mavlink)
target_link_libraries(objs/AP_AIS/ArduSub dronecan)
target_link_libraries(objs/AP_AIS/ArduSub littlefs)

set(objs/AP_Arming/ArduSub_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs/AP_Arming/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Arming/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Arming/ArduSub ${objs/AP_Arming/ArduSub_SOURCES})

target_link_libraries(objs/AP_Arming/ArduSub mavlink)
target_link_libraries(objs/AP_Arming/ArduSub dronecan)
target_link_libraries(objs/AP_Arming/ArduSub littlefs)

set(objs/AP_Follow/ArduSub_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs/AP_Follow/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Follow/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Follow/ArduSub ${objs/AP_Follow/ArduSub_SOURCES})

target_link_libraries(objs/AP_Follow/ArduSub mavlink)
target_link_libraries(objs/AP_Follow/ArduSub dronecan)
target_link_libraries(objs/AP_Follow/ArduSub littlefs)

set(objs/AC_AttitudeControl/ArduSub_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs/AC_AttitudeControl/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_AttitudeControl/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AC_AttitudeControl/ArduSub ${objs/AC_AttitudeControl/ArduSub_SOURCES})

target_link_libraries(objs/AC_AttitudeControl/ArduSub mavlink)
target_link_libraries(objs/AC_AttitudeControl/ArduSub dronecan)
target_link_libraries(objs/AC_AttitudeControl/ArduSub littlefs)

set(objs/AC_WPNav/ArduSub_SOURCES
    ../libraries/AC_WPNav/AC_Loiter.cpp)

set(objs/AC_WPNav/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_WPNav/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AC_WPNav/ArduSub ${objs/AC_WPNav/ArduSub_SOURCES})

target_link_libraries(objs/AC_WPNav/ArduSub mavlink)
target_link_libraries(objs/AC_WPNav/ArduSub dronecan)
target_link_libraries(objs/AC_WPNav/ArduSub littlefs)

set(objs/AP_Motors/ArduSub_SOURCES
    ../libraries/AP_Motors/AP_MotorsMatrix.cpp
    ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    ../libraries/AP_Motors/AP_MotorsTri.cpp
    ../libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp)

set(objs/AP_Motors/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Motors/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Motors/ArduSub ${objs/AP_Motors/ArduSub_SOURCES})

target_link_libraries(objs/AP_Motors/ArduSub mavlink)
target_link_libraries(objs/AP_Motors/ArduSub dronecan)
target_link_libraries(objs/AP_Motors/ArduSub littlefs)

set(objs/AP_HAL_SITL/ArduSub_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs/AP_HAL_SITL/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_HAL_SITL/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_HAL_SITL/ArduSub ${objs/AP_HAL_SITL/ArduSub_SOURCES})

target_link_libraries(objs/AP_HAL_SITL/ArduSub mavlink)
target_link_libraries(objs/AP_HAL_SITL/ArduSub dronecan)
target_link_libraries(objs/AP_HAL_SITL/ArduSub littlefs)

set(objs/SITL/ArduSub_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs/SITL/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/SITL/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/SITL/ArduSub ${objs/SITL/ArduSub_SOURCES})

target_link_libraries(objs/SITL/ArduSub mavlink)
target_link_libraries(objs/SITL/ArduSub dronecan)
target_link_libraries(objs/SITL/ArduSub littlefs)

set(objs/AP_Scripting/ArduSub_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs/AP_Scripting/ArduSub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Scripting/ArduSub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_library(objs/AP_Scripting/ArduSub ${objs/AP_Scripting/ArduSub_SOURCES})

target_link_libraries(objs/AP_Scripting/ArduSub mavlink)
target_link_libraries(objs/AP_Scripting/ArduSub dronecan)
target_link_libraries(objs/AP_Scripting/ArduSub littlefs)

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

set(bin/ardusub_SOURCES
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

set(bin/ardusub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${bin/ardusub_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub")

add_executable(bin/ardusub ${bin/ardusub_SOURCES})

target_link_libraries(bin/ardusub ArduSub_libs)

