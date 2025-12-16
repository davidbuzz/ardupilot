
set(objs_AP_AHRS_ArduPlane_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs_AP_AHRS_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_AHRS_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_AHRS_ArduPlane ${objs_AP_AHRS_ArduPlane_SOURCES})
set_target_properties(objs_AP_AHRS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_ArduPlane mavlink)
target_link_libraries(objs_AP_AHRS_ArduPlane dronecan)
target_link_libraries(objs_AP_AHRS_ArduPlane littlefs)

set(objs_AP_Airspeed_ArduPlane_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs_AP_Airspeed_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Airspeed_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Airspeed_ArduPlane ${objs_AP_Airspeed_ArduPlane_SOURCES})
set_target_properties(objs_AP_Airspeed_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_ArduPlane mavlink)
target_link_libraries(objs_AP_Airspeed_ArduPlane dronecan)
target_link_libraries(objs_AP_Airspeed_ArduPlane littlefs)

set(objs_AP_Baro_ArduPlane_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs_AP_Baro_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Baro_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Baro_ArduPlane ${objs_AP_Baro_ArduPlane_SOURCES})
set_target_properties(objs_AP_Baro_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_ArduPlane mavlink)
target_link_libraries(objs_AP_Baro_ArduPlane dronecan)
target_link_libraries(objs_AP_Baro_ArduPlane littlefs)

set(objs_AP_BattMonitor_ArduPlane_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs_AP_BattMonitor_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BattMonitor_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_BattMonitor_ArduPlane ${objs_AP_BattMonitor_ArduPlane_SOURCES})
set_target_properties(objs_AP_BattMonitor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_ArduPlane mavlink)
target_link_libraries(objs_AP_BattMonitor_ArduPlane dronecan)
target_link_libraries(objs_AP_BattMonitor_ArduPlane littlefs)

set(objs_AP_BoardConfig_ArduPlane_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs_AP_BoardConfig_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BoardConfig_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_BoardConfig_ArduPlane ${objs_AP_BoardConfig_ArduPlane_SOURCES})
set_target_properties(objs_AP_BoardConfig_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_ArduPlane mavlink)
target_link_libraries(objs_AP_BoardConfig_ArduPlane dronecan)
target_link_libraries(objs_AP_BoardConfig_ArduPlane littlefs)

set(objs_AP_HAL_ArduPlane_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs_AP_HAL_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_HAL_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_HAL_ArduPlane ${objs_AP_HAL_ArduPlane_SOURCES})
set_target_properties(objs_AP_HAL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_ArduPlane mavlink)
target_link_libraries(objs_AP_HAL_ArduPlane dronecan)
target_link_libraries(objs_AP_HAL_ArduPlane littlefs)

set(objs_AP_InertialSensor_ArduPlane_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs_AP_InertialSensor_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_InertialSensor_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_InertialSensor_ArduPlane ${objs_AP_InertialSensor_ArduPlane_SOURCES})
set_target_properties(objs_AP_InertialSensor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_ArduPlane mavlink)
target_link_libraries(objs_AP_InertialSensor_ArduPlane dronecan)
target_link_libraries(objs_AP_InertialSensor_ArduPlane littlefs)

set(objs_AP_Math_ArduPlane_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs_AP_Math_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Math_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Math_ArduPlane ${objs_AP_Math_ArduPlane_SOURCES})
set_target_properties(objs_AP_Math_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_ArduPlane mavlink)
target_link_libraries(objs_AP_Math_ArduPlane dronecan)
target_link_libraries(objs_AP_Math_ArduPlane littlefs)

set(objs_AP_Mission_ArduPlane_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs_AP_Mission_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Mission_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Mission_ArduPlane ${objs_AP_Mission_ArduPlane_SOURCES})
set_target_properties(objs_AP_Mission_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_ArduPlane mavlink)
target_link_libraries(objs_AP_Mission_ArduPlane dronecan)
target_link_libraries(objs_AP_Mission_ArduPlane littlefs)

set(objs_AP_DAL_ArduPlane_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs_AP_DAL_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_DAL_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_DAL_ArduPlane ${objs_AP_DAL_ArduPlane_SOURCES})
set_target_properties(objs_AP_DAL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_ArduPlane mavlink)
target_link_libraries(objs_AP_DAL_ArduPlane dronecan)
target_link_libraries(objs_AP_DAL_ArduPlane littlefs)

set(objs_AP_NavEKF2_ArduPlane_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs_AP_NavEKF2_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_NavEKF2_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_NavEKF2_ArduPlane ${objs_AP_NavEKF2_ArduPlane_SOURCES})
set_target_properties(objs_AP_NavEKF2_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_ArduPlane mavlink)
target_link_libraries(objs_AP_NavEKF2_ArduPlane dronecan)
target_link_libraries(objs_AP_NavEKF2_ArduPlane littlefs)

set(objs_AP_NavEKF3_ArduPlane_SOURCES
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

set(objs_AP_NavEKF3_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_NavEKF3_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_NavEKF3_ArduPlane ${objs_AP_NavEKF3_ArduPlane_SOURCES})
set_target_properties(objs_AP_NavEKF3_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_ArduPlane mavlink)
target_link_libraries(objs_AP_NavEKF3_ArduPlane dronecan)
target_link_libraries(objs_AP_NavEKF3_ArduPlane littlefs)

set(objs_AP_Rally_ArduPlane_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs_AP_Rally_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Rally_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Rally_ArduPlane ${objs_AP_Rally_ArduPlane_SOURCES})
set_target_properties(objs_AP_Rally_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_ArduPlane mavlink)
target_link_libraries(objs_AP_Rally_ArduPlane dronecan)
target_link_libraries(objs_AP_Rally_ArduPlane littlefs)

set(objs_AP_Scheduler_ArduPlane_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs_AP_Scheduler_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Scheduler_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Scheduler_ArduPlane ${objs_AP_Scheduler_ArduPlane_SOURCES})
set_target_properties(objs_AP_Scheduler_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_ArduPlane mavlink)
target_link_libraries(objs_AP_Scheduler_ArduPlane dronecan)
target_link_libraries(objs_AP_Scheduler_ArduPlane littlefs)

set(objs_AP_Terrain_ArduPlane_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs_AP_Terrain_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Terrain_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Terrain_ArduPlane ${objs_AP_Terrain_ArduPlane_SOURCES})
set_target_properties(objs_AP_Terrain_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_ArduPlane mavlink)
target_link_libraries(objs_AP_Terrain_ArduPlane dronecan)
target_link_libraries(objs_AP_Terrain_ArduPlane littlefs)

set(objs_AP_Vehicle_ArduPlane_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs_AP_Vehicle_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Vehicle_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Vehicle_ArduPlane ${objs_AP_Vehicle_ArduPlane_SOURCES})
set_target_properties(objs_AP_Vehicle_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_ArduPlane mavlink)
target_link_libraries(objs_AP_Vehicle_ArduPlane dronecan)
target_link_libraries(objs_AP_Vehicle_ArduPlane littlefs)

set(objs_AP_Logger_ArduPlane_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs_AP_Logger_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Logger_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Logger_ArduPlane ${objs_AP_Logger_ArduPlane_SOURCES})
set_target_properties(objs_AP_Logger_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_ArduPlane mavlink)
target_link_libraries(objs_AP_Logger_ArduPlane dronecan)
target_link_libraries(objs_AP_Logger_ArduPlane littlefs)

set(objs_Filter_ArduPlane_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs_Filter_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_Filter_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_Filter_ArduPlane ${objs_Filter_ArduPlane_SOURCES})
set_target_properties(objs_Filter_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_ArduPlane mavlink)
target_link_libraries(objs_Filter_ArduPlane dronecan)
target_link_libraries(objs_Filter_ArduPlane littlefs)

set(objs_GCS_MAVLink_ArduPlane_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs_GCS_MAVLink_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_GCS_MAVLink_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_GCS_MAVLink_ArduPlane ${objs_GCS_MAVLink_ArduPlane_SOURCES})
set_target_properties(objs_GCS_MAVLink_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_ArduPlane mavlink)
target_link_libraries(objs_GCS_MAVLink_ArduPlane dronecan)
target_link_libraries(objs_GCS_MAVLink_ArduPlane littlefs)

set(objs_RC_Channel_ArduPlane_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs_RC_Channel_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_RC_Channel_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_RC_Channel_ArduPlane ${objs_RC_Channel_ArduPlane_SOURCES})
set_target_properties(objs_RC_Channel_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_ArduPlane mavlink)
target_link_libraries(objs_RC_Channel_ArduPlane dronecan)
target_link_libraries(objs_RC_Channel_ArduPlane littlefs)

set(objs_StorageManager_ArduPlane_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs_StorageManager_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_StorageManager_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_StorageManager_ArduPlane ${objs_StorageManager_ArduPlane_SOURCES})
set_target_properties(objs_StorageManager_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_ArduPlane mavlink)
target_link_libraries(objs_StorageManager_ArduPlane dronecan)
target_link_libraries(objs_StorageManager_ArduPlane littlefs)

set(objs_AP_Frsky_Telem_ArduPlane_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs_AP_Frsky_Telem_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Frsky_Telem_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Frsky_Telem_ArduPlane ${objs_AP_Frsky_Telem_ArduPlane_SOURCES})
set_target_properties(objs_AP_Frsky_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_ArduPlane mavlink)
target_link_libraries(objs_AP_Frsky_Telem_ArduPlane dronecan)
target_link_libraries(objs_AP_Frsky_Telem_ArduPlane littlefs)

set(objs_AP_IBus_Telem_ArduPlane_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs_AP_IBus_Telem_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_IBus_Telem_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_IBus_Telem_ArduPlane ${objs_AP_IBus_Telem_ArduPlane_SOURCES})
set_target_properties(objs_AP_IBus_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_ArduPlane mavlink)
target_link_libraries(objs_AP_IBus_Telem_ArduPlane dronecan)
target_link_libraries(objs_AP_IBus_Telem_ArduPlane littlefs)

set(objs_AP_Relay_ArduPlane_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs_AP_Relay_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Relay_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Relay_ArduPlane ${objs_AP_Relay_ArduPlane_SOURCES})
set_target_properties(objs_AP_Relay_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_ArduPlane mavlink)
target_link_libraries(objs_AP_Relay_ArduPlane dronecan)
target_link_libraries(objs_AP_Relay_ArduPlane littlefs)

set(objs_AP_RCProtocol_ArduPlane_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs_AP_RCProtocol_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_RCProtocol_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_RCProtocol_ArduPlane ${objs_AP_RCProtocol_ArduPlane_SOURCES})
set_target_properties(objs_AP_RCProtocol_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_ArduPlane mavlink)
target_link_libraries(objs_AP_RCProtocol_ArduPlane dronecan)
target_link_libraries(objs_AP_RCProtocol_ArduPlane littlefs)

set(objs_AP_BLHeli_ArduPlane_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs_AP_BLHeli_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BLHeli_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_BLHeli_ArduPlane ${objs_AP_BLHeli_ArduPlane_SOURCES})
set_target_properties(objs_AP_BLHeli_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_ArduPlane mavlink)
target_link_libraries(objs_AP_BLHeli_ArduPlane dronecan)
target_link_libraries(objs_AP_BLHeli_ArduPlane littlefs)

set(objs_AP_Proximity_ArduPlane_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs_AP_Proximity_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Proximity_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Proximity_ArduPlane ${objs_AP_Proximity_ArduPlane_SOURCES})
set_target_properties(objs_AP_Proximity_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_ArduPlane mavlink)
target_link_libraries(objs_AP_Proximity_ArduPlane dronecan)
target_link_libraries(objs_AP_Proximity_ArduPlane littlefs)

set(objs_AC_Fence_ArduPlane_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs_AC_Fence_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_Fence_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AC_Fence_ArduPlane ${objs_AC_Fence_ArduPlane_SOURCES})
set_target_properties(objs_AC_Fence_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_ArduPlane mavlink)
target_link_libraries(objs_AC_Fence_ArduPlane dronecan)
target_link_libraries(objs_AC_Fence_ArduPlane littlefs)

set(objs_AC_Avoidance_ArduPlane_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs_AC_Avoidance_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_Avoidance_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AC_Avoidance_ArduPlane ${objs_AC_Avoidance_ArduPlane_SOURCES})
set_target_properties(objs_AC_Avoidance_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_ArduPlane mavlink)
target_link_libraries(objs_AC_Avoidance_ArduPlane dronecan)
target_link_libraries(objs_AC_Avoidance_ArduPlane littlefs)

set(objs_AP_LandingGear_ArduPlane_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs_AP_LandingGear_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_LandingGear_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_LandingGear_ArduPlane ${objs_AP_LandingGear_ArduPlane_SOURCES})
set_target_properties(objs_AP_LandingGear_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_ArduPlane mavlink)
target_link_libraries(objs_AP_LandingGear_ArduPlane dronecan)
target_link_libraries(objs_AP_LandingGear_ArduPlane littlefs)

set(objs_AP_OSD_ArduPlane_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs_AP_OSD_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_OSD_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_OSD_ArduPlane ${objs_AP_OSD_ArduPlane_SOURCES})
set_target_properties(objs_AP_OSD_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_ArduPlane mavlink)
target_link_libraries(objs_AP_OSD_ArduPlane dronecan)
target_link_libraries(objs_AP_OSD_ArduPlane littlefs)

set(objs_AP_Filesystem_ArduPlane_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs_AP_Filesystem_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Filesystem_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Filesystem_ArduPlane ${objs_AP_Filesystem_ArduPlane_SOURCES})
set_target_properties(objs_AP_Filesystem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_ArduPlane mavlink)
target_link_libraries(objs_AP_Filesystem_ArduPlane dronecan)
target_link_libraries(objs_AP_Filesystem_ArduPlane littlefs)

set(objs_AP_ADSB_ArduPlane_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs_AP_ADSB_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ADSB_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_ADSB_ArduPlane ${objs_AP_ADSB_ArduPlane_SOURCES})
set_target_properties(objs_AP_ADSB_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_ArduPlane mavlink)
target_link_libraries(objs_AP_ADSB_ArduPlane dronecan)
target_link_libraries(objs_AP_ADSB_ArduPlane littlefs)

set(objs_AP_ESC_Telem_ArduPlane_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs_AP_ESC_Telem_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ESC_Telem_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_ESC_Telem_ArduPlane ${objs_AP_ESC_Telem_ArduPlane_SOURCES})
set_target_properties(objs_AP_ESC_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_ArduPlane mavlink)
target_link_libraries(objs_AP_ESC_Telem_ArduPlane dronecan)
target_link_libraries(objs_AP_ESC_Telem_ArduPlane littlefs)

set(objs_AP_GyroFFT_ArduPlane_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs_AP_GyroFFT_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_GyroFFT_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_GyroFFT_ArduPlane ${objs_AP_GyroFFT_ArduPlane_SOURCES})
set_target_properties(objs_AP_GyroFFT_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_ArduPlane mavlink)
target_link_libraries(objs_AP_GyroFFT_ArduPlane dronecan)
target_link_libraries(objs_AP_GyroFFT_ArduPlane littlefs)

set(objs_AP_RCTelemetry_ArduPlane_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs_AP_RCTelemetry_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_RCTelemetry_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_RCTelemetry_ArduPlane ${objs_AP_RCTelemetry_ArduPlane_SOURCES})
set_target_properties(objs_AP_RCTelemetry_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_ArduPlane mavlink)
target_link_libraries(objs_AP_RCTelemetry_ArduPlane dronecan)
target_link_libraries(objs_AP_RCTelemetry_ArduPlane littlefs)

set(objs_AP_ExternalAHRS_ArduPlane_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs_AP_ExternalAHRS_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ExternalAHRS_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_ExternalAHRS_ArduPlane ${objs_AP_ExternalAHRS_ArduPlane_SOURCES})
set_target_properties(objs_AP_ExternalAHRS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_ArduPlane mavlink)
target_link_libraries(objs_AP_ExternalAHRS_ArduPlane dronecan)
target_link_libraries(objs_AP_ExternalAHRS_ArduPlane littlefs)

set(objs_AP_TemperatureSensor_ArduPlane_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs_AP_TemperatureSensor_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_TemperatureSensor_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_TemperatureSensor_ArduPlane ${objs_AP_TemperatureSensor_ArduPlane_SOURCES})
set_target_properties(objs_AP_TemperatureSensor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_ArduPlane mavlink)
target_link_libraries(objs_AP_TemperatureSensor_ArduPlane dronecan)
target_link_libraries(objs_AP_TemperatureSensor_ArduPlane littlefs)

set(objs_AP_AIS_ArduPlane_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs_AP_AIS_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_AIS_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_AIS_ArduPlane ${objs_AP_AIS_ArduPlane_SOURCES})
set_target_properties(objs_AP_AIS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_ArduPlane mavlink)
target_link_libraries(objs_AP_AIS_ArduPlane dronecan)
target_link_libraries(objs_AP_AIS_ArduPlane littlefs)

set(objs_AP_Arming_ArduPlane_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs_AP_Arming_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Arming_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Arming_ArduPlane ${objs_AP_Arming_ArduPlane_SOURCES})
set_target_properties(objs_AP_Arming_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_ArduPlane mavlink)
target_link_libraries(objs_AP_Arming_ArduPlane dronecan)
target_link_libraries(objs_AP_Arming_ArduPlane littlefs)

set(objs_AP_Follow_ArduPlane_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs_AP_Follow_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Follow_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Follow_ArduPlane ${objs_AP_Follow_ArduPlane_SOURCES})
set_target_properties(objs_AP_Follow_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_ArduPlane mavlink)
target_link_libraries(objs_AP_Follow_ArduPlane dronecan)
target_link_libraries(objs_AP_Follow_ArduPlane littlefs)

set(objs_AP_Avoidance_ArduPlane_SOURCES
    ../libraries/AP_Avoidance/AP_Avoidance.cpp)

set(objs_AP_Avoidance_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Avoidance_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Avoidance_ArduPlane ${objs_AP_Avoidance_ArduPlane_SOURCES})
set_target_properties(objs_AP_Avoidance_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Avoidance_ArduPlane mavlink)
target_link_libraries(objs_AP_Avoidance_ArduPlane dronecan)
target_link_libraries(objs_AP_Avoidance_ArduPlane littlefs)

set(objs_AC_WPNav_ArduPlane_SOURCES
    ../libraries/AC_WPNav/AC_Loiter.cpp)

set(objs_AC_WPNav_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_WPNav_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AC_WPNav_ArduPlane ${objs_AC_WPNav_ArduPlane_SOURCES})
set_target_properties(objs_AC_WPNav_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav_ArduPlane mavlink)
target_link_libraries(objs_AC_WPNav_ArduPlane dronecan)
target_link_libraries(objs_AC_WPNav_ArduPlane littlefs)

set(objs_AC_AttitudeControl_ArduPlane_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs_AC_AttitudeControl_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_AttitudeControl_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AC_AttitudeControl_ArduPlane ${objs_AC_AttitudeControl_ArduPlane_SOURCES})
set_target_properties(objs_AC_AttitudeControl_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_ArduPlane mavlink)
target_link_libraries(objs_AC_AttitudeControl_ArduPlane dronecan)
target_link_libraries(objs_AC_AttitudeControl_ArduPlane littlefs)

set(objs_AP_Motors_ArduPlane_SOURCES
    ../libraries/AP_Motors/AP_MotorsMatrix.cpp
    ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    ../libraries/AP_Motors/AP_MotorsTri.cpp
    ../libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp)

set(objs_AP_Motors_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Motors_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Motors_ArduPlane ${objs_AP_Motors_ArduPlane_SOURCES})
set_target_properties(objs_AP_Motors_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors_ArduPlane mavlink)
target_link_libraries(objs_AP_Motors_ArduPlane dronecan)
target_link_libraries(objs_AP_Motors_ArduPlane littlefs)

set(objs_AP_Landing_ArduPlane_SOURCES
    ../libraries/AP_Landing/AP_Landing_Slope.cpp)

set(objs_AP_Landing_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Landing_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Landing_ArduPlane ${objs_AP_Landing_ArduPlane_SOURCES})
set_target_properties(objs_AP_Landing_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Landing_ArduPlane mavlink)
target_link_libraries(objs_AP_Landing_ArduPlane dronecan)
target_link_libraries(objs_AP_Landing_ArduPlane littlefs)

set(objs_AC_AutoTune_ArduPlane_SOURCES
    ../libraries/AC_AutoTune/AC_AutoTune.cpp
    ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp)

set(objs_AC_AutoTune_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_AutoTune_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AC_AutoTune_ArduPlane ${objs_AC_AutoTune_ArduPlane_SOURCES})
set_target_properties(objs_AC_AutoTune_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AutoTune_ArduPlane mavlink)
target_link_libraries(objs_AC_AutoTune_ArduPlane dronecan)
target_link_libraries(objs_AC_AutoTune_ArduPlane littlefs)

set(objs_AC_PrecLand_ArduPlane_SOURCES
    ../libraries/AC_PrecLand/AC_PrecLand.cpp)

set(objs_AC_PrecLand_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_PrecLand_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AC_PrecLand_ArduPlane ${objs_AC_PrecLand_ArduPlane_SOURCES})
set_target_properties(objs_AC_PrecLand_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_PrecLand_ArduPlane mavlink)
target_link_libraries(objs_AC_PrecLand_ArduPlane dronecan)
target_link_libraries(objs_AC_PrecLand_ArduPlane littlefs)

set(objs_AP_HAL_SITL_ArduPlane_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs_AP_HAL_SITL_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_HAL_SITL_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_HAL_SITL_ArduPlane ${objs_AP_HAL_SITL_ArduPlane_SOURCES})
set_target_properties(objs_AP_HAL_SITL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_ArduPlane mavlink)
target_link_libraries(objs_AP_HAL_SITL_ArduPlane dronecan)
target_link_libraries(objs_AP_HAL_SITL_ArduPlane littlefs)

set(objs_SITL_ArduPlane_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs_SITL_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_SITL_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_SITL_ArduPlane ${objs_SITL_ArduPlane_SOURCES})
set_target_properties(objs_SITL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_ArduPlane mavlink)
target_link_libraries(objs_SITL_ArduPlane dronecan)
target_link_libraries(objs_SITL_ArduPlane littlefs)

set(objs_AP_Scripting_ArduPlane_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs_AP_Scripting_ArduPlane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Scripting_ArduPlane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_library(objs_AP_Scripting_ArduPlane ${objs_AP_Scripting_ArduPlane_SOURCES})
set_target_properties(objs_AP_Scripting_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_ArduPlane mavlink)
target_link_libraries(objs_AP_Scripting_ArduPlane dronecan)
target_link_libraries(objs_AP_Scripting_ArduPlane littlefs)

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
set_target_properties(ArduPlane_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


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

set(bin_arduplane_SOURCES
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

set(bin_arduplane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${bin_arduplane_INCLUDES})

add_definitions(-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane")

add_executable(bin_arduplane ${bin_arduplane_SOURCES})

target_link_libraries(bin_arduplane ArduPlane_libs)

