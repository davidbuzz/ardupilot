
set(objs/AP_AHRS/ArduCopter_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs/AP_AHRS/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_AHRS/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_AHRS/ArduCopter ${objs/AP_AHRS/ArduCopter_SOURCES})

target_link_libraries(objs/AP_AHRS/ArduCopter mavlink)
target_link_libraries(objs/AP_AHRS/ArduCopter dronecan)
target_link_libraries(objs/AP_AHRS/ArduCopter littlefs)

set(objs/AP_Airspeed/ArduCopter_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs/AP_Airspeed/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Airspeed/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Airspeed/ArduCopter ${objs/AP_Airspeed/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Airspeed/ArduCopter mavlink)
target_link_libraries(objs/AP_Airspeed/ArduCopter dronecan)
target_link_libraries(objs/AP_Airspeed/ArduCopter littlefs)

set(objs/AP_Baro/ArduCopter_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs/AP_Baro/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Baro/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Baro/ArduCopter ${objs/AP_Baro/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Baro/ArduCopter mavlink)
target_link_libraries(objs/AP_Baro/ArduCopter dronecan)
target_link_libraries(objs/AP_Baro/ArduCopter littlefs)

set(objs/AP_BattMonitor/ArduCopter_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs/AP_BattMonitor/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_BattMonitor/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_BattMonitor/ArduCopter ${objs/AP_BattMonitor/ArduCopter_SOURCES})

target_link_libraries(objs/AP_BattMonitor/ArduCopter mavlink)
target_link_libraries(objs/AP_BattMonitor/ArduCopter dronecan)
target_link_libraries(objs/AP_BattMonitor/ArduCopter littlefs)

set(objs/AP_BoardConfig/ArduCopter_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs/AP_BoardConfig/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_BoardConfig/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_BoardConfig/ArduCopter ${objs/AP_BoardConfig/ArduCopter_SOURCES})

target_link_libraries(objs/AP_BoardConfig/ArduCopter mavlink)
target_link_libraries(objs/AP_BoardConfig/ArduCopter dronecan)
target_link_libraries(objs/AP_BoardConfig/ArduCopter littlefs)

set(objs/AP_HAL/ArduCopter_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs/AP_HAL/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_HAL/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_HAL/ArduCopter ${objs/AP_HAL/ArduCopter_SOURCES})

target_link_libraries(objs/AP_HAL/ArduCopter mavlink)
target_link_libraries(objs/AP_HAL/ArduCopter dronecan)
target_link_libraries(objs/AP_HAL/ArduCopter littlefs)

set(objs/AP_InertialSensor/ArduCopter_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs/AP_InertialSensor/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_InertialSensor/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_InertialSensor/ArduCopter ${objs/AP_InertialSensor/ArduCopter_SOURCES})

target_link_libraries(objs/AP_InertialSensor/ArduCopter mavlink)
target_link_libraries(objs/AP_InertialSensor/ArduCopter dronecan)
target_link_libraries(objs/AP_InertialSensor/ArduCopter littlefs)

set(objs/AP_Math/ArduCopter_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs/AP_Math/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Math/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Math/ArduCopter ${objs/AP_Math/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Math/ArduCopter mavlink)
target_link_libraries(objs/AP_Math/ArduCopter dronecan)
target_link_libraries(objs/AP_Math/ArduCopter littlefs)

set(objs/AP_Mission/ArduCopter_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs/AP_Mission/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Mission/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Mission/ArduCopter ${objs/AP_Mission/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Mission/ArduCopter mavlink)
target_link_libraries(objs/AP_Mission/ArduCopter dronecan)
target_link_libraries(objs/AP_Mission/ArduCopter littlefs)

set(objs/AP_DAL/ArduCopter_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs/AP_DAL/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_DAL/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_DAL/ArduCopter ${objs/AP_DAL/ArduCopter_SOURCES})

target_link_libraries(objs/AP_DAL/ArduCopter mavlink)
target_link_libraries(objs/AP_DAL/ArduCopter dronecan)
target_link_libraries(objs/AP_DAL/ArduCopter littlefs)

set(objs/AP_NavEKF2/ArduCopter_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs/AP_NavEKF2/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_NavEKF2/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_NavEKF2/ArduCopter ${objs/AP_NavEKF2/ArduCopter_SOURCES})

target_link_libraries(objs/AP_NavEKF2/ArduCopter mavlink)
target_link_libraries(objs/AP_NavEKF2/ArduCopter dronecan)
target_link_libraries(objs/AP_NavEKF2/ArduCopter littlefs)

set(objs/AP_NavEKF3/ArduCopter_SOURCES
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

set(objs/AP_NavEKF3/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_NavEKF3/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_NavEKF3/ArduCopter ${objs/AP_NavEKF3/ArduCopter_SOURCES})

target_link_libraries(objs/AP_NavEKF3/ArduCopter mavlink)
target_link_libraries(objs/AP_NavEKF3/ArduCopter dronecan)
target_link_libraries(objs/AP_NavEKF3/ArduCopter littlefs)

set(objs/AP_Rally/ArduCopter_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs/AP_Rally/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Rally/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Rally/ArduCopter ${objs/AP_Rally/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Rally/ArduCopter mavlink)
target_link_libraries(objs/AP_Rally/ArduCopter dronecan)
target_link_libraries(objs/AP_Rally/ArduCopter littlefs)

set(objs/AP_Scheduler/ArduCopter_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs/AP_Scheduler/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Scheduler/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Scheduler/ArduCopter ${objs/AP_Scheduler/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Scheduler/ArduCopter mavlink)
target_link_libraries(objs/AP_Scheduler/ArduCopter dronecan)
target_link_libraries(objs/AP_Scheduler/ArduCopter littlefs)

set(objs/AP_Terrain/ArduCopter_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs/AP_Terrain/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Terrain/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Terrain/ArduCopter ${objs/AP_Terrain/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Terrain/ArduCopter mavlink)
target_link_libraries(objs/AP_Terrain/ArduCopter dronecan)
target_link_libraries(objs/AP_Terrain/ArduCopter littlefs)

set(objs/AP_Vehicle/ArduCopter_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs/AP_Vehicle/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Vehicle/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Vehicle/ArduCopter ${objs/AP_Vehicle/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Vehicle/ArduCopter mavlink)
target_link_libraries(objs/AP_Vehicle/ArduCopter dronecan)
target_link_libraries(objs/AP_Vehicle/ArduCopter littlefs)

set(objs/AP_Logger/ArduCopter_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs/AP_Logger/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Logger/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Logger/ArduCopter ${objs/AP_Logger/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Logger/ArduCopter mavlink)
target_link_libraries(objs/AP_Logger/ArduCopter dronecan)
target_link_libraries(objs/AP_Logger/ArduCopter littlefs)

set(objs/Filter/ArduCopter_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs/Filter/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/Filter/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/Filter/ArduCopter ${objs/Filter/ArduCopter_SOURCES})

target_link_libraries(objs/Filter/ArduCopter mavlink)
target_link_libraries(objs/Filter/ArduCopter dronecan)
target_link_libraries(objs/Filter/ArduCopter littlefs)

set(objs/GCS_MAVLink/ArduCopter_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs/GCS_MAVLink/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/GCS_MAVLink/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/GCS_MAVLink/ArduCopter ${objs/GCS_MAVLink/ArduCopter_SOURCES})

target_link_libraries(objs/GCS_MAVLink/ArduCopter mavlink)
target_link_libraries(objs/GCS_MAVLink/ArduCopter dronecan)
target_link_libraries(objs/GCS_MAVLink/ArduCopter littlefs)

set(objs/RC_Channel/ArduCopter_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs/RC_Channel/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/RC_Channel/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/RC_Channel/ArduCopter ${objs/RC_Channel/ArduCopter_SOURCES})

target_link_libraries(objs/RC_Channel/ArduCopter mavlink)
target_link_libraries(objs/RC_Channel/ArduCopter dronecan)
target_link_libraries(objs/RC_Channel/ArduCopter littlefs)

set(objs/StorageManager/ArduCopter_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs/StorageManager/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/StorageManager/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/StorageManager/ArduCopter ${objs/StorageManager/ArduCopter_SOURCES})

target_link_libraries(objs/StorageManager/ArduCopter mavlink)
target_link_libraries(objs/StorageManager/ArduCopter dronecan)
target_link_libraries(objs/StorageManager/ArduCopter littlefs)

set(objs/AP_Frsky_Telem/ArduCopter_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs/AP_Frsky_Telem/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Frsky_Telem/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Frsky_Telem/ArduCopter ${objs/AP_Frsky_Telem/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Frsky_Telem/ArduCopter mavlink)
target_link_libraries(objs/AP_Frsky_Telem/ArduCopter dronecan)
target_link_libraries(objs/AP_Frsky_Telem/ArduCopter littlefs)

set(objs/AP_IBus_Telem/ArduCopter_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs/AP_IBus_Telem/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_IBus_Telem/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_IBus_Telem/ArduCopter ${objs/AP_IBus_Telem/ArduCopter_SOURCES})

target_link_libraries(objs/AP_IBus_Telem/ArduCopter mavlink)
target_link_libraries(objs/AP_IBus_Telem/ArduCopter dronecan)
target_link_libraries(objs/AP_IBus_Telem/ArduCopter littlefs)

set(objs/AP_Relay/ArduCopter_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs/AP_Relay/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Relay/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Relay/ArduCopter ${objs/AP_Relay/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Relay/ArduCopter mavlink)
target_link_libraries(objs/AP_Relay/ArduCopter dronecan)
target_link_libraries(objs/AP_Relay/ArduCopter littlefs)

set(objs/AP_RCProtocol/ArduCopter_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs/AP_RCProtocol/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_RCProtocol/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_RCProtocol/ArduCopter ${objs/AP_RCProtocol/ArduCopter_SOURCES})

target_link_libraries(objs/AP_RCProtocol/ArduCopter mavlink)
target_link_libraries(objs/AP_RCProtocol/ArduCopter dronecan)
target_link_libraries(objs/AP_RCProtocol/ArduCopter littlefs)

set(objs/AP_BLHeli/ArduCopter_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs/AP_BLHeli/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_BLHeli/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_BLHeli/ArduCopter ${objs/AP_BLHeli/ArduCopter_SOURCES})

target_link_libraries(objs/AP_BLHeli/ArduCopter mavlink)
target_link_libraries(objs/AP_BLHeli/ArduCopter dronecan)
target_link_libraries(objs/AP_BLHeli/ArduCopter littlefs)

set(objs/AP_Proximity/ArduCopter_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs/AP_Proximity/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Proximity/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Proximity/ArduCopter ${objs/AP_Proximity/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Proximity/ArduCopter mavlink)
target_link_libraries(objs/AP_Proximity/ArduCopter dronecan)
target_link_libraries(objs/AP_Proximity/ArduCopter littlefs)

set(objs/AC_Fence/ArduCopter_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs/AC_Fence/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_Fence/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AC_Fence/ArduCopter ${objs/AC_Fence/ArduCopter_SOURCES})

target_link_libraries(objs/AC_Fence/ArduCopter mavlink)
target_link_libraries(objs/AC_Fence/ArduCopter dronecan)
target_link_libraries(objs/AC_Fence/ArduCopter littlefs)

set(objs/AC_Avoidance/ArduCopter_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs/AC_Avoidance/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_Avoidance/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AC_Avoidance/ArduCopter ${objs/AC_Avoidance/ArduCopter_SOURCES})

target_link_libraries(objs/AC_Avoidance/ArduCopter mavlink)
target_link_libraries(objs/AC_Avoidance/ArduCopter dronecan)
target_link_libraries(objs/AC_Avoidance/ArduCopter littlefs)

set(objs/AP_LandingGear/ArduCopter_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs/AP_LandingGear/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_LandingGear/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_LandingGear/ArduCopter ${objs/AP_LandingGear/ArduCopter_SOURCES})

target_link_libraries(objs/AP_LandingGear/ArduCopter mavlink)
target_link_libraries(objs/AP_LandingGear/ArduCopter dronecan)
target_link_libraries(objs/AP_LandingGear/ArduCopter littlefs)

set(objs/AP_OSD/ArduCopter_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs/AP_OSD/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_OSD/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_OSD/ArduCopter ${objs/AP_OSD/ArduCopter_SOURCES})

target_link_libraries(objs/AP_OSD/ArduCopter mavlink)
target_link_libraries(objs/AP_OSD/ArduCopter dronecan)
target_link_libraries(objs/AP_OSD/ArduCopter littlefs)

set(objs/AP_Filesystem/ArduCopter_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs/AP_Filesystem/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Filesystem/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Filesystem/ArduCopter ${objs/AP_Filesystem/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Filesystem/ArduCopter mavlink)
target_link_libraries(objs/AP_Filesystem/ArduCopter dronecan)
target_link_libraries(objs/AP_Filesystem/ArduCopter littlefs)

set(objs/AP_ADSB/ArduCopter_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs/AP_ADSB/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_ADSB/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_ADSB/ArduCopter ${objs/AP_ADSB/ArduCopter_SOURCES})

target_link_libraries(objs/AP_ADSB/ArduCopter mavlink)
target_link_libraries(objs/AP_ADSB/ArduCopter dronecan)
target_link_libraries(objs/AP_ADSB/ArduCopter littlefs)

set(objs/AP_ESC_Telem/ArduCopter_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs/AP_ESC_Telem/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_ESC_Telem/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_ESC_Telem/ArduCopter ${objs/AP_ESC_Telem/ArduCopter_SOURCES})

target_link_libraries(objs/AP_ESC_Telem/ArduCopter mavlink)
target_link_libraries(objs/AP_ESC_Telem/ArduCopter dronecan)
target_link_libraries(objs/AP_ESC_Telem/ArduCopter littlefs)

set(objs/AP_GyroFFT/ArduCopter_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs/AP_GyroFFT/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_GyroFFT/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_GyroFFT/ArduCopter ${objs/AP_GyroFFT/ArduCopter_SOURCES})

target_link_libraries(objs/AP_GyroFFT/ArduCopter mavlink)
target_link_libraries(objs/AP_GyroFFT/ArduCopter dronecan)
target_link_libraries(objs/AP_GyroFFT/ArduCopter littlefs)

set(objs/AP_RCTelemetry/ArduCopter_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs/AP_RCTelemetry/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_RCTelemetry/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_RCTelemetry/ArduCopter ${objs/AP_RCTelemetry/ArduCopter_SOURCES})

target_link_libraries(objs/AP_RCTelemetry/ArduCopter mavlink)
target_link_libraries(objs/AP_RCTelemetry/ArduCopter dronecan)
target_link_libraries(objs/AP_RCTelemetry/ArduCopter littlefs)

set(objs/AP_ExternalAHRS/ArduCopter_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs/AP_ExternalAHRS/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_ExternalAHRS/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_ExternalAHRS/ArduCopter ${objs/AP_ExternalAHRS/ArduCopter_SOURCES})

target_link_libraries(objs/AP_ExternalAHRS/ArduCopter mavlink)
target_link_libraries(objs/AP_ExternalAHRS/ArduCopter dronecan)
target_link_libraries(objs/AP_ExternalAHRS/ArduCopter littlefs)

set(objs/AP_TemperatureSensor/ArduCopter_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs/AP_TemperatureSensor/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_TemperatureSensor/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_TemperatureSensor/ArduCopter ${objs/AP_TemperatureSensor/ArduCopter_SOURCES})

target_link_libraries(objs/AP_TemperatureSensor/ArduCopter mavlink)
target_link_libraries(objs/AP_TemperatureSensor/ArduCopter dronecan)
target_link_libraries(objs/AP_TemperatureSensor/ArduCopter littlefs)

set(objs/AP_AIS/ArduCopter_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs/AP_AIS/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_AIS/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_AIS/ArduCopter ${objs/AP_AIS/ArduCopter_SOURCES})

target_link_libraries(objs/AP_AIS/ArduCopter mavlink)
target_link_libraries(objs/AP_AIS/ArduCopter dronecan)
target_link_libraries(objs/AP_AIS/ArduCopter littlefs)

set(objs/AP_Arming/ArduCopter_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs/AP_Arming/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Arming/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Arming/ArduCopter ${objs/AP_Arming/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Arming/ArduCopter mavlink)
target_link_libraries(objs/AP_Arming/ArduCopter dronecan)
target_link_libraries(objs/AP_Arming/ArduCopter littlefs)

set(objs/AP_Follow/ArduCopter_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs/AP_Follow/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Follow/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Follow/ArduCopter ${objs/AP_Follow/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Follow/ArduCopter mavlink)
target_link_libraries(objs/AP_Follow/ArduCopter dronecan)
target_link_libraries(objs/AP_Follow/ArduCopter littlefs)

set(objs/AC_AttitudeControl/ArduCopter_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs/AC_AttitudeControl/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_AttitudeControl/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AC_AttitudeControl/ArduCopter ${objs/AC_AttitudeControl/ArduCopter_SOURCES})

target_link_libraries(objs/AC_AttitudeControl/ArduCopter mavlink)
target_link_libraries(objs/AC_AttitudeControl/ArduCopter dronecan)
target_link_libraries(objs/AC_AttitudeControl/ArduCopter littlefs)

set(objs/AC_PrecLand/ArduCopter_SOURCES
    ../libraries/AC_PrecLand/AC_PrecLand.cpp)

set(objs/AC_PrecLand/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_PrecLand/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AC_PrecLand/ArduCopter ${objs/AC_PrecLand/ArduCopter_SOURCES})

target_link_libraries(objs/AC_PrecLand/ArduCopter mavlink)
target_link_libraries(objs/AC_PrecLand/ArduCopter dronecan)
target_link_libraries(objs/AC_PrecLand/ArduCopter littlefs)

set(objs/AC_WPNav/ArduCopter_SOURCES
    ../libraries/AC_WPNav/AC_Loiter.cpp)

set(objs/AC_WPNav/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_WPNav/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AC_WPNav/ArduCopter ${objs/AC_WPNav/ArduCopter_SOURCES})

target_link_libraries(objs/AC_WPNav/ArduCopter mavlink)
target_link_libraries(objs/AC_WPNav/ArduCopter dronecan)
target_link_libraries(objs/AC_WPNav/ArduCopter littlefs)

set(objs/AP_Motors/ArduCopter_SOURCES
    ../libraries/AP_Motors/AP_MotorsMatrix.cpp
    ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    ../libraries/AP_Motors/AP_MotorsTri.cpp
    ../libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp)

set(objs/AP_Motors/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Motors/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Motors/ArduCopter ${objs/AP_Motors/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Motors/ArduCopter mavlink)
target_link_libraries(objs/AP_Motors/ArduCopter dronecan)
target_link_libraries(objs/AP_Motors/ArduCopter littlefs)

set(objs/AP_Avoidance/ArduCopter_SOURCES
    ../libraries/AP_Avoidance/AP_Avoidance.cpp)

set(objs/AP_Avoidance/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Avoidance/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Avoidance/ArduCopter ${objs/AP_Avoidance/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Avoidance/ArduCopter mavlink)
target_link_libraries(objs/AP_Avoidance/ArduCopter dronecan)
target_link_libraries(objs/AP_Avoidance/ArduCopter littlefs)

set(objs/AC_AutoTune/ArduCopter_SOURCES
    ../libraries/AC_AutoTune/AC_AutoTune.cpp
    ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp)

set(objs/AC_AutoTune/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AC_AutoTune/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AC_AutoTune/ArduCopter ${objs/AC_AutoTune/ArduCopter_SOURCES})

target_link_libraries(objs/AC_AutoTune/ArduCopter mavlink)
target_link_libraries(objs/AC_AutoTune/ArduCopter dronecan)
target_link_libraries(objs/AC_AutoTune/ArduCopter littlefs)

set(objs/AP_HAL_SITL/ArduCopter_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs/AP_HAL_SITL/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_HAL_SITL/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_HAL_SITL/ArduCopter ${objs/AP_HAL_SITL/ArduCopter_SOURCES})

target_link_libraries(objs/AP_HAL_SITL/ArduCopter mavlink)
target_link_libraries(objs/AP_HAL_SITL/ArduCopter dronecan)
target_link_libraries(objs/AP_HAL_SITL/ArduCopter littlefs)

set(objs/SITL/ArduCopter_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs/SITL/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/SITL/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/SITL/ArduCopter ${objs/SITL/ArduCopter_SOURCES})

target_link_libraries(objs/SITL/ArduCopter mavlink)
target_link_libraries(objs/SITL/ArduCopter dronecan)
target_link_libraries(objs/SITL/ArduCopter littlefs)

set(objs/AP_Scripting/ArduCopter_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs/AP_Scripting/ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs/AP_Scripting/ArduCopter_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_library(objs/AP_Scripting/ArduCopter ${objs/AP_Scripting/ArduCopter_SOURCES})

target_link_libraries(objs/AP_Scripting/ArduCopter mavlink)
target_link_libraries(objs/AP_Scripting/ArduCopter dronecan)
target_link_libraries(objs/AP_Scripting/ArduCopter littlefs)

set(ArduCopter_libs_SOURCES
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

set(ArduCopter_libs_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${ArduCopter_libs_INCLUDES})

add_library(ArduCopter_libs ${ArduCopter_libs_SOURCES})

target_link_libraries(ArduCopter_libs objs/AP_AccelCal)
target_link_libraries(ArduCopter_libs objs/AP_AccelCal/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ADC)
target_link_libraries(ArduCopter_libs objs/AP_ADC/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_AHRS)
target_link_libraries(ArduCopter_libs objs/AP_AHRS/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Airspeed)
target_link_libraries(ArduCopter_libs objs/AP_Airspeed/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Baro)
target_link_libraries(ArduCopter_libs objs/AP_Baro/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_BattMonitor)
target_link_libraries(ArduCopter_libs objs/AP_BattMonitor/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_BoardConfig)
target_link_libraries(ArduCopter_libs objs/AP_BoardConfig/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Camera)
target_link_libraries(ArduCopter_libs objs/AP_Camera/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Common)
target_link_libraries(ArduCopter_libs objs/AP_Common/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Compass)
target_link_libraries(ArduCopter_libs objs/AP_Compass/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Declination)
target_link_libraries(ArduCopter_libs objs/AP_Declination/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_GPS)
target_link_libraries(ArduCopter_libs objs/AP_GPS/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_GSOF)
target_link_libraries(ArduCopter_libs objs/AP_GSOF/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_HAL)
target_link_libraries(ArduCopter_libs objs/AP_HAL/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_HAL_Empty)
target_link_libraries(ArduCopter_libs objs/AP_HAL_Empty/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_DDS)
target_link_libraries(ArduCopter_libs objs/AP_DDS/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_InertialSensor)
target_link_libraries(ArduCopter_libs objs/AP_InertialSensor/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Math)
target_link_libraries(ArduCopter_libs objs/AP_Math/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Mission)
target_link_libraries(ArduCopter_libs objs/AP_Mission/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_DAL)
target_link_libraries(ArduCopter_libs objs/AP_DAL/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_NavEKF)
target_link_libraries(ArduCopter_libs objs/AP_NavEKF/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_NavEKF2)
target_link_libraries(ArduCopter_libs objs/AP_NavEKF2/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_NavEKF3)
target_link_libraries(ArduCopter_libs objs/AP_NavEKF3/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Notify)
target_link_libraries(ArduCopter_libs objs/AP_Notify/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_OpticalFlow)
target_link_libraries(ArduCopter_libs objs/AP_OpticalFlow/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Param)
target_link_libraries(ArduCopter_libs objs/AP_Param/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Rally)
target_link_libraries(ArduCopter_libs objs/AP_Rally/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_LightWareSerial)
target_link_libraries(ArduCopter_libs objs/AP_LightWareSerial/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RangeFinder)
target_link_libraries(ArduCopter_libs objs/AP_RangeFinder/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Scheduler)
target_link_libraries(ArduCopter_libs objs/AP_Scheduler/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_SerialManager)
target_link_libraries(ArduCopter_libs objs/AP_SerialManager/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Terrain)
target_link_libraries(ArduCopter_libs objs/AP_Terrain/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Vehicle)
target_link_libraries(ArduCopter_libs objs/AP_Vehicle/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_InternalError)
target_link_libraries(ArduCopter_libs objs/AP_InternalError/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Logger)
target_link_libraries(ArduCopter_libs objs/AP_Logger/ArduCopter)
target_link_libraries(ArduCopter_libs objs/Filter)
target_link_libraries(ArduCopter_libs objs/Filter/ArduCopter)
target_link_libraries(ArduCopter_libs objs/GCS_MAVLink)
target_link_libraries(ArduCopter_libs objs/GCS_MAVLink/ArduCopter)
target_link_libraries(ArduCopter_libs objs/RC_Channel)
target_link_libraries(ArduCopter_libs objs/RC_Channel/ArduCopter)
target_link_libraries(ArduCopter_libs objs/SRV_Channel)
target_link_libraries(ArduCopter_libs objs/SRV_Channel/ArduCopter)
target_link_libraries(ArduCopter_libs objs/StorageManager)
target_link_libraries(ArduCopter_libs objs/StorageManager/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Tuning)
target_link_libraries(ArduCopter_libs objs/AP_Tuning/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RPM)
target_link_libraries(ArduCopter_libs objs/AP_RPM/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RSSI)
target_link_libraries(ArduCopter_libs objs/AP_RSSI/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Mount)
target_link_libraries(ArduCopter_libs objs/AP_Mount/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Module)
target_link_libraries(ArduCopter_libs objs/AP_Module/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Button)
target_link_libraries(ArduCopter_libs objs/AP_Button/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ICEngine)
target_link_libraries(ArduCopter_libs objs/AP_ICEngine/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Networking)
target_link_libraries(ArduCopter_libs objs/AP_Networking/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Frsky_Telem)
target_link_libraries(ArduCopter_libs objs/AP_Frsky_Telem/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_IBus_Telem)
target_link_libraries(ArduCopter_libs objs/AP_IBus_Telem/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_FlashStorage)
target_link_libraries(ArduCopter_libs objs/AP_FlashStorage/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Relay)
target_link_libraries(ArduCopter_libs objs/AP_Relay/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ServoRelayEvents)
target_link_libraries(ArduCopter_libs objs/AP_ServoRelayEvents/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Volz_Protocol)
target_link_libraries(ArduCopter_libs objs/AP_Volz_Protocol/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_SBusOut)
target_link_libraries(ArduCopter_libs objs/AP_SBusOut/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_IOMCU)
target_link_libraries(ArduCopter_libs objs/AP_IOMCU/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Parachute)
target_link_libraries(ArduCopter_libs objs/AP_Parachute/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RAMTRON)
target_link_libraries(ArduCopter_libs objs/AP_RAMTRON/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RCProtocol)
target_link_libraries(ArduCopter_libs objs/AP_RCProtocol/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Radio)
target_link_libraries(ArduCopter_libs objs/AP_Radio/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_TempCalibration)
target_link_libraries(ArduCopter_libs objs/AP_TempCalibration/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_VisualOdom)
target_link_libraries(ArduCopter_libs objs/AP_VisualOdom/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_BLHeli)
target_link_libraries(ArduCopter_libs objs/AP_BLHeli/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ROMFS)
target_link_libraries(ArduCopter_libs objs/AP_ROMFS/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Proximity)
target_link_libraries(ArduCopter_libs objs/AP_Proximity/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Gripper)
target_link_libraries(ArduCopter_libs objs/AP_Gripper/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RTC)
target_link_libraries(ArduCopter_libs objs/AP_RTC/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_Sprayer)
target_link_libraries(ArduCopter_libs objs/AC_Sprayer/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_Fence)
target_link_libraries(ArduCopter_libs objs/AC_Fence/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_Avoidance)
target_link_libraries(ArduCopter_libs objs/AC_Avoidance/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_LandingGear)
target_link_libraries(ArduCopter_libs objs/AP_LandingGear/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RobotisServo)
target_link_libraries(ArduCopter_libs objs/AP_RobotisServo/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_NMEA_Output)
target_link_libraries(ArduCopter_libs objs/AP_NMEA_Output/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_OSD)
target_link_libraries(ArduCopter_libs objs/AP_OSD/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Filesystem)
target_link_libraries(ArduCopter_libs objs/AP_Filesystem/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ADSB)
target_link_libraries(ArduCopter_libs objs/AP_ADSB/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ADSB/sagetech-sdk)
target_link_libraries(ArduCopter_libs objs/AP_ADSB/sagetech-sdk/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_PID)
target_link_libraries(ArduCopter_libs objs/AC_PID/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_SerialLED)
target_link_libraries(ArduCopter_libs objs/AP_SerialLED/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_EFI)
target_link_libraries(ArduCopter_libs objs/AP_EFI/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Hott_Telem)
target_link_libraries(ArduCopter_libs objs/AP_Hott_Telem/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ESC_Telem)
target_link_libraries(ArduCopter_libs objs/AP_ESC_Telem/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Servo_Telem)
target_link_libraries(ArduCopter_libs objs/AP_Servo_Telem/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Stats)
target_link_libraries(ArduCopter_libs objs/AP_Stats/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_GyroFFT)
target_link_libraries(ArduCopter_libs objs/AP_GyroFFT/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RCTelemetry)
target_link_libraries(ArduCopter_libs objs/AP_RCTelemetry/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Generator)
target_link_libraries(ArduCopter_libs objs/AP_Generator/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_MSP)
target_link_libraries(ArduCopter_libs objs/AP_MSP/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_OLC)
target_link_libraries(ArduCopter_libs objs/AP_OLC/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_WheelEncoder)
target_link_libraries(ArduCopter_libs objs/AP_WheelEncoder/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ExternalAHRS)
target_link_libraries(ArduCopter_libs objs/AP_ExternalAHRS/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_VideoTX)
target_link_libraries(ArduCopter_libs objs/AP_VideoTX/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_FETtecOneWire)
target_link_libraries(ArduCopter_libs objs/AP_FETtecOneWire/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_TemperatureSensor)
target_link_libraries(ArduCopter_libs objs/AP_TemperatureSensor/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Torqeedo)
target_link_libraries(ArduCopter_libs objs/AP_Torqeedo/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_CustomRotations)
target_link_libraries(ArduCopter_libs objs/AP_CustomRotations/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_AIS)
target_link_libraries(ArduCopter_libs objs/AP_AIS/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_OpenDroneID)
target_link_libraries(ArduCopter_libs objs/AP_OpenDroneID/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_CheckFirmware)
target_link_libraries(ArduCopter_libs objs/AP_CheckFirmware/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_ExternalControl)
target_link_libraries(ArduCopter_libs objs/AP_ExternalControl/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_JSON)
target_link_libraries(ArduCopter_libs objs/AP_JSON/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Beacon)
target_link_libraries(ArduCopter_libs objs/AP_Beacon/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Arming)
target_link_libraries(ArduCopter_libs objs/AP_Arming/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_RCMapper)
target_link_libraries(ArduCopter_libs objs/AP_RCMapper/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_MultiHeap)
target_link_libraries(ArduCopter_libs objs/AP_MultiHeap/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Follow)
target_link_libraries(ArduCopter_libs objs/AP_Follow/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_CANManager)
target_link_libraries(ArduCopter_libs objs/AP_CANManager/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_KDECAN)
target_link_libraries(ArduCopter_libs objs/AP_KDECAN/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_PiccoloCAN)
target_link_libraries(ArduCopter_libs objs/AP_PiccoloCAN/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_PiccoloCAN/piccolo_protocol)
target_link_libraries(ArduCopter_libs objs/AP_PiccoloCAN/piccolo_protocol/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_AttitudeControl)
target_link_libraries(ArduCopter_libs objs/AC_AttitudeControl/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_InputManager)
target_link_libraries(ArduCopter_libs objs/AC_InputManager/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_PrecLand)
target_link_libraries(ArduCopter_libs objs/AC_PrecLand/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_Autorotation)
target_link_libraries(ArduCopter_libs objs/AC_Autorotation/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_WPNav)
target_link_libraries(ArduCopter_libs objs/AC_WPNav/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_IRLock)
target_link_libraries(ArduCopter_libs objs/AP_IRLock/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Motors)
target_link_libraries(ArduCopter_libs objs/AP_Motors/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Avoidance)
target_link_libraries(ArduCopter_libs objs/AP_Avoidance/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_AdvancedFailsafe)
target_link_libraries(ArduCopter_libs objs/AP_AdvancedFailsafe/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_SmartRTL)
target_link_libraries(ArduCopter_libs objs/AP_SmartRTL/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Winch)
target_link_libraries(ArduCopter_libs objs/AP_Winch/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_LTM_Telem)
target_link_libraries(ArduCopter_libs objs/AP_LTM_Telem/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Devo_Telem)
target_link_libraries(ArduCopter_libs objs/AP_Devo_Telem/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_AutoTune)
target_link_libraries(ArduCopter_libs objs/AC_AutoTune/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_SurfaceDistance)
target_link_libraries(ArduCopter_libs objs/AP_SurfaceDistance/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_DroneCAN)
target_link_libraries(ArduCopter_libs objs/AP_DroneCAN/ArduCopter)
target_link_libraries(ArduCopter_libs objs/modules/DroneCAN/libcanard/*.c)
target_link_libraries(ArduCopter_libs objs/modules/DroneCAN/libcanard/*.c/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_HAL_SITL)
target_link_libraries(ArduCopter_libs objs/AP_HAL_SITL/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_CSVReader)
target_link_libraries(ArduCopter_libs objs/AP_CSVReader/ArduCopter)
target_link_libraries(ArduCopter_libs objs/SITL)
target_link_libraries(ArduCopter_libs objs/SITL/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Scripting)
target_link_libraries(ArduCopter_libs objs/AP_Scripting/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AP_Scripting/lua/src)
target_link_libraries(ArduCopter_libs objs/AP_Scripting/lua/src/ArduCopter)
target_link_libraries(ArduCopter_libs objs/AC_CustomControl)
target_link_libraries(ArduCopter_libs objs/AC_CustomControl/ArduCopter)

set(bin/arducopter_SOURCES
    AP_Arming_Copter.cpp
    AP_ExternalControl_Copter.cpp
    AP_Rally.cpp
    AP_State.cpp
    Attitude.cpp
    Copter.cpp
    GCS_Copter.cpp
    GCS_MAVLink_Copter.cpp
    Log.cpp
    Parameters.cpp
    RC_Channel_Copter.cpp
    UserCode.cpp
    UserParameters.cpp
    afs_copter.cpp
    autoyaw.cpp
    avoidance.cpp
    avoidance_adsb.cpp
    baro_ground_effect.cpp
    commands.cpp
    compassmot.cpp
    crash_check.cpp
    ekf_check.cpp
    esc_calibration.cpp
    events.cpp
    failsafe.cpp
    fence.cpp
    heli.cpp
    inertia.cpp
    land_detector.cpp
    landing_gear.cpp
    mode.cpp
    mode_acro.cpp
    mode_acro_heli.cpp
    mode_althold.cpp
    mode_auto.cpp
    mode_autorotate.cpp
    mode_autotune.cpp
    mode_avoid_adsb.cpp
    mode_brake.cpp
    mode_circle.cpp
    mode_drift.cpp
    mode_flip.cpp
    mode_flowhold.cpp
    mode_follow.cpp
    mode_guided.cpp
    mode_guided_custom.cpp
    mode_guided_nogps.cpp
    mode_land.cpp
    mode_loiter.cpp
    mode_poshold.cpp
    mode_rtl.cpp
    mode_smart_rtl.cpp
    mode_sport.cpp
    mode_stabilize.cpp
    mode_stabilize_heli.cpp
    mode_systemid.cpp
    mode_throw.cpp
    mode_turtle.cpp
    mode_zigzag.cpp
    motor_test.cpp
    motors.cpp
    navigation.cpp
    precision_landing.cpp
    radio.cpp
    rate_thread.cpp
    sensors.cpp
    standby.cpp
    surface_tracking.cpp
    system.cpp
    takeoff.cpp
    takeoff_check.cpp
    terrain.cpp
    toy_mode.cpp
    tuning.cpp)

set(bin/arducopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${bin/arducopter_INCLUDES})

add_definitions(-DFRAME_CONFIG=MULTICOPTER_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_executable(bin/arducopter ${bin/arducopter_SOURCES})

target_link_libraries(bin/arducopter A)
target_link_libraries(bin/arducopter r)
target_link_libraries(bin/arducopter d)
target_link_libraries(bin/arducopter u)
target_link_libraries(bin/arducopter C)
target_link_libraries(bin/arducopter o)
target_link_libraries(bin/arducopter p)
target_link_libraries(bin/arducopter t)
target_link_libraries(bin/arducopter e)
target_link_libraries(bin/arducopter r)
target_link_libraries(bin/arducopter _)
target_link_libraries(bin/arducopter l)
target_link_libraries(bin/arducopter i)
target_link_libraries(bin/arducopter b)
target_link_libraries(bin/arducopter s)

set(bin/arducopter-heli_SOURCES
    AP_Arming_Copter.cpp
    AP_ExternalControl_Copter.cpp
    AP_Rally.cpp
    AP_State.cpp
    Attitude.cpp
    Copter.cpp
    GCS_Copter.cpp
    GCS_MAVLink_Copter.cpp
    Log.cpp
    Parameters.cpp
    RC_Channel_Copter.cpp
    UserCode.cpp
    UserParameters.cpp
    afs_copter.cpp
    autoyaw.cpp
    avoidance.cpp
    avoidance_adsb.cpp
    baro_ground_effect.cpp
    commands.cpp
    compassmot.cpp
    crash_check.cpp
    ekf_check.cpp
    esc_calibration.cpp
    events.cpp
    failsafe.cpp
    fence.cpp
    heli.cpp
    inertia.cpp
    land_detector.cpp
    landing_gear.cpp
    mode.cpp
    mode_acro.cpp
    mode_acro_heli.cpp
    mode_althold.cpp
    mode_auto.cpp
    mode_autorotate.cpp
    mode_autotune.cpp
    mode_avoid_adsb.cpp
    mode_brake.cpp
    mode_circle.cpp
    mode_drift.cpp
    mode_flip.cpp
    mode_flowhold.cpp
    mode_follow.cpp
    mode_guided.cpp
    mode_guided_custom.cpp
    mode_guided_nogps.cpp
    mode_land.cpp
    mode_loiter.cpp
    mode_poshold.cpp
    mode_rtl.cpp
    mode_smart_rtl.cpp
    mode_sport.cpp
    mode_stabilize.cpp
    mode_stabilize_heli.cpp
    mode_systemid.cpp
    mode_throw.cpp
    mode_turtle.cpp
    mode_zigzag.cpp
    motor_test.cpp
    motors.cpp
    navigation.cpp
    precision_landing.cpp
    radio.cpp
    rate_thread.cpp
    sensors.cpp
    standby.cpp
    surface_tracking.cpp
    system.cpp
    takeoff.cpp
    takeoff_check.cpp
    terrain.cpp
    toy_mode.cpp
    tuning.cpp)

set(bin/arducopter-heli_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${bin/arducopter-heli_INCLUDES})

add_definitions(-DFRAME_CONFIG=HELI_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter")

add_executable(bin/arducopter-heli ${bin/arducopter-heli_SOURCES})

target_link_libraries(bin/arducopter-heli A)
target_link_libraries(bin/arducopter-heli r)
target_link_libraries(bin/arducopter-heli d)
target_link_libraries(bin/arducopter-heli u)
target_link_libraries(bin/arducopter-heli C)
target_link_libraries(bin/arducopter-heli o)
target_link_libraries(bin/arducopter-heli p)
target_link_libraries(bin/arducopter-heli t)
target_link_libraries(bin/arducopter-heli e)
target_link_libraries(bin/arducopter-heli r)
target_link_libraries(bin/arducopter-heli _)
target_link_libraries(bin/arducopter-heli l)
target_link_libraries(bin/arducopter-heli i)
target_link_libraries(bin/arducopter-heli b)
target_link_libraries(bin/arducopter-heli s)

