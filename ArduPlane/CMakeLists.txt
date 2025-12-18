
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

add_library(objs_AP_AHRS_ArduPlane ${objs_AP_AHRS_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_AHRS_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_AHRS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_AHRS_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_AHRS_ArduPlane littlefs) #clean

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

add_library(objs_AP_Airspeed_ArduPlane ${objs_AP_Airspeed_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Airspeed_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Airspeed_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Airspeed_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Airspeed_ArduPlane littlefs) #clean

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

add_library(objs_AP_Baro_ArduPlane ${objs_AP_Baro_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Baro_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Baro_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Baro_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Baro_ArduPlane littlefs) #clean

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

add_library(objs_AP_BattMonitor_ArduPlane ${objs_AP_BattMonitor_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_BattMonitor_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_BattMonitor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_ArduPlane littlefs) #clean

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

add_library(objs_AP_BoardConfig_ArduPlane ${objs_AP_BoardConfig_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_BoardConfig_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_BoardConfig_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_ArduPlane littlefs) #clean

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

add_library(objs_AP_HAL_ArduPlane ${objs_AP_HAL_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_HAL_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_HAL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_HAL_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_HAL_ArduPlane littlefs) #clean

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

add_library(objs_AP_InertialSensor_ArduPlane ${objs_AP_InertialSensor_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_InertialSensor_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_InertialSensor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_ArduPlane littlefs) #clean

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

add_library(objs_AP_Math_ArduPlane ${objs_AP_Math_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Math_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Math_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Math_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Math_ArduPlane littlefs) #clean

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

add_library(objs_AP_Mission_ArduPlane ${objs_AP_Mission_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Mission_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Mission_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Mission_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Mission_ArduPlane littlefs) #clean

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

add_library(objs_AP_DAL_ArduPlane ${objs_AP_DAL_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_DAL_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_DAL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_DAL_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_DAL_ArduPlane littlefs) #clean

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

add_library(objs_AP_NavEKF2_ArduPlane ${objs_AP_NavEKF2_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_NavEKF2_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_NavEKF2_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_ArduPlane littlefs) #clean

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

add_library(objs_AP_NavEKF3_ArduPlane ${objs_AP_NavEKF3_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_NavEKF3_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_NavEKF3_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_ArduPlane littlefs) #clean

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

add_library(objs_AP_Rally_ArduPlane ${objs_AP_Rally_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Rally_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Rally_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Rally_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Rally_ArduPlane littlefs) #clean

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

add_library(objs_AP_Scheduler_ArduPlane ${objs_AP_Scheduler_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Scheduler_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Scheduler_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Scheduler_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Scheduler_ArduPlane littlefs) #clean

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

add_library(objs_AP_Terrain_ArduPlane ${objs_AP_Terrain_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Terrain_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Terrain_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Terrain_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Terrain_ArduPlane littlefs) #clean

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

add_library(objs_AP_Vehicle_ArduPlane ${objs_AP_Vehicle_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Vehicle_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Vehicle_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Vehicle_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Vehicle_ArduPlane littlefs) #clean

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

add_library(objs_AP_Logger_ArduPlane ${objs_AP_Logger_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Logger_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Logger_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Logger_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Logger_ArduPlane littlefs) #clean

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

add_library(objs_Filter_ArduPlane ${objs_Filter_ArduPlane_SOURCES})
target_compile_definitions(objs_Filter_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_Filter_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_ArduPlane mavlink) #clean
target_link_libraries(objs_Filter_ArduPlane dronecan) #clean
target_link_libraries(objs_Filter_ArduPlane littlefs) #clean

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

add_library(objs_GCS_MAVLink_ArduPlane ${objs_GCS_MAVLink_ArduPlane_SOURCES})
target_compile_definitions(objs_GCS_MAVLink_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_GCS_MAVLink_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_ArduPlane mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_ArduPlane dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_ArduPlane littlefs) #clean

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

add_library(objs_RC_Channel_ArduPlane ${objs_RC_Channel_ArduPlane_SOURCES})
target_compile_definitions(objs_RC_Channel_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_RC_Channel_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_ArduPlane mavlink) #clean
target_link_libraries(objs_RC_Channel_ArduPlane dronecan) #clean
target_link_libraries(objs_RC_Channel_ArduPlane littlefs) #clean

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

add_library(objs_StorageManager_ArduPlane ${objs_StorageManager_ArduPlane_SOURCES})
target_compile_definitions(objs_StorageManager_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_StorageManager_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_ArduPlane mavlink) #clean
target_link_libraries(objs_StorageManager_ArduPlane dronecan) #clean
target_link_libraries(objs_StorageManager_ArduPlane littlefs) #clean

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

add_library(objs_AP_Frsky_Telem_ArduPlane ${objs_AP_Frsky_Telem_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Frsky_Telem_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Frsky_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduPlane littlefs) #clean

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

add_library(objs_AP_IBus_Telem_ArduPlane ${objs_AP_IBus_Telem_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_IBus_Telem_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_IBus_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduPlane littlefs) #clean

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

add_library(objs_AP_Relay_ArduPlane ${objs_AP_Relay_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Relay_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Relay_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Relay_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Relay_ArduPlane littlefs) #clean

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

add_library(objs_AP_RCProtocol_ArduPlane ${objs_AP_RCProtocol_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_RCProtocol_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_RCProtocol_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_ArduPlane littlefs) #clean

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

add_library(objs_AP_BLHeli_ArduPlane ${objs_AP_BLHeli_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_BLHeli_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_BLHeli_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_BLHeli_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_BLHeli_ArduPlane littlefs) #clean

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

add_library(objs_AP_Proximity_ArduPlane ${objs_AP_Proximity_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Proximity_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Proximity_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Proximity_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Proximity_ArduPlane littlefs) #clean

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

add_library(objs_AC_Fence_ArduPlane ${objs_AC_Fence_ArduPlane_SOURCES})
target_compile_definitions(objs_AC_Fence_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AC_Fence_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_Fence_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_Fence_ArduPlane littlefs) #clean

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

add_library(objs_AC_Avoidance_ArduPlane ${objs_AC_Avoidance_ArduPlane_SOURCES})
target_compile_definitions(objs_AC_Avoidance_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AC_Avoidance_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_Avoidance_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_Avoidance_ArduPlane littlefs) #clean

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

add_library(objs_AP_LandingGear_ArduPlane ${objs_AP_LandingGear_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_LandingGear_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_LandingGear_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_LandingGear_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_LandingGear_ArduPlane littlefs) #clean

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

add_library(objs_AP_OSD_ArduPlane ${objs_AP_OSD_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_OSD_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_OSD_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_OSD_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_OSD_ArduPlane littlefs) #clean

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

add_library(objs_AP_Filesystem_ArduPlane ${objs_AP_Filesystem_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Filesystem_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Filesystem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Filesystem_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Filesystem_ArduPlane littlefs) #clean

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

add_library(objs_AP_ADSB_ArduPlane ${objs_AP_ADSB_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_ADSB_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_ADSB_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_ADSB_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_ADSB_ArduPlane littlefs) #clean

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

add_library(objs_AP_ESC_Telem_ArduPlane ${objs_AP_ESC_Telem_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_ESC_Telem_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_ESC_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduPlane littlefs) #clean

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

add_library(objs_AP_GyroFFT_ArduPlane ${objs_AP_GyroFFT_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_GyroFFT_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_GyroFFT_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_ArduPlane littlefs) #clean

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

add_library(objs_AP_RCTelemetry_ArduPlane ${objs_AP_RCTelemetry_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_RCTelemetry_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_RCTelemetry_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduPlane littlefs) #clean

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

add_library(objs_AP_ExternalAHRS_ArduPlane ${objs_AP_ExternalAHRS_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_ExternalAHRS_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_ExternalAHRS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduPlane littlefs) #clean

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

add_library(objs_AP_TemperatureSensor_ArduPlane ${objs_AP_TemperatureSensor_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_TemperatureSensor_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_TemperatureSensor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduPlane littlefs) #clean

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

add_library(objs_AP_AIS_ArduPlane ${objs_AP_AIS_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_AIS_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_AIS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_AIS_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_AIS_ArduPlane littlefs) #clean

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

add_library(objs_AP_Arming_ArduPlane ${objs_AP_Arming_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Arming_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Arming_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Arming_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Arming_ArduPlane littlefs) #clean

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

add_library(objs_AP_Follow_ArduPlane ${objs_AP_Follow_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Follow_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Follow_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Follow_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Follow_ArduPlane littlefs) #clean

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

add_library(objs_AP_Avoidance_ArduPlane ${objs_AP_Avoidance_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Avoidance_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Avoidance_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Avoidance_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Avoidance_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Avoidance_ArduPlane littlefs) #clean

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

add_library(objs_AC_WPNav_ArduPlane ${objs_AC_WPNav_ArduPlane_SOURCES})
target_compile_definitions(objs_AC_WPNav_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AC_WPNav_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_WPNav_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_WPNav_ArduPlane littlefs) #clean

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

add_library(objs_AC_AttitudeControl_ArduPlane ${objs_AC_AttitudeControl_ArduPlane_SOURCES})
target_compile_definitions(objs_AC_AttitudeControl_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AC_AttitudeControl_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduPlane littlefs) #clean

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

add_library(objs_AP_Motors_ArduPlane ${objs_AP_Motors_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Motors_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Motors_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Motors_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Motors_ArduPlane littlefs) #clean

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

add_library(objs_AP_Landing_ArduPlane ${objs_AP_Landing_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Landing_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Landing_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Landing_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Landing_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Landing_ArduPlane littlefs) #clean

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

add_library(objs_AC_AutoTune_ArduPlane ${objs_AC_AutoTune_ArduPlane_SOURCES})
target_compile_definitions(objs_AC_AutoTune_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AC_AutoTune_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AutoTune_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_AutoTune_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_AutoTune_ArduPlane littlefs) #clean

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

add_library(objs_AC_PrecLand_ArduPlane ${objs_AC_PrecLand_ArduPlane_SOURCES})
target_compile_definitions(objs_AC_PrecLand_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AC_PrecLand_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_PrecLand_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_PrecLand_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_PrecLand_ArduPlane littlefs) #clean

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

add_library(objs_AP_HAL_SITL_ArduPlane ${objs_AP_HAL_SITL_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_HAL_SITL_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_HAL_SITL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduPlane littlefs) #clean

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

add_library(objs_SITL_ArduPlane ${objs_SITL_ArduPlane_SOURCES})
target_compile_definitions(objs_SITL_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_SITL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_ArduPlane mavlink) #clean
target_link_libraries(objs_SITL_ArduPlane dronecan) #clean
target_link_libraries(objs_SITL_ArduPlane littlefs) #clean

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

add_library(objs_AP_Scripting_ArduPlane ${objs_AP_Scripting_ArduPlane_SOURCES})
target_compile_definitions(objs_AP_Scripting_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2

set_target_properties(objs_AP_Scripting_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Scripting_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Scripting_ArduPlane littlefs) #clean

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


target_link_libraries(ArduPlane_libs objs_AP_AccelCal_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_ADC_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_AHRS_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Airspeed_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Baro_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_BattMonitor_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_BoardConfig_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Camera_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Common_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Compass_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Declination_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_GPS_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_GSOF_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_HAL_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_HAL_Empty_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_DDS_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_InertialSensor_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Math_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Mission_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_DAL_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_NavEKF_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_NavEKF2_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_NavEKF3_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Notify_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_OpticalFlow_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Param_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Rally_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_LightWareSerial_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RangeFinder_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Scheduler_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_SerialManager_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Terrain_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Vehicle_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_InternalError_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Logger_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_Filter_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_GCS_MAVLink_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_RC_Channel_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_SRV_Channel_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_StorageManager_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Tuning_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RPM_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RSSI_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Mount_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Module_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Button_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_ICEngine_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Networking_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Frsky_Telem_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_IBus_Telem_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_FlashStorage_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Relay_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_ServoRelayEvents_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Volz_Protocol_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_SBusOut_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_IOMCU_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Parachute_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RAMTRON_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RCProtocol_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Radio_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_TempCalibration_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_VisualOdom_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_BLHeli_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_ROMFS_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Proximity_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Gripper_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RTC_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_Sprayer_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_Fence_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_Avoidance_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_LandingGear_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RobotisServo_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_NMEA_Output_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_OSD_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Filesystem_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_ADSB_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_PID_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_SerialLED_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_EFI_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Hott_Telem_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_ESC_Telem_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Servo_Telem_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Stats_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_GyroFFT_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RCTelemetry_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Generator_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_MSP_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_OLC_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_WheelEncoder_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_ExternalAHRS_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_VideoTX_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_FETtecOneWire_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_TemperatureSensor_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Torqeedo_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_CustomRotations_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_AIS_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_OpenDroneID_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_CheckFirmware_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_ExternalControl_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_JSON_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Beacon_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Arming_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_RCMapper_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_MultiHeap_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Follow_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_CANManager_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_KDECAN_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_PiccoloCAN_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_APM_Control_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_AdvancedFailsafe_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Avoidance_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_L1_Control_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Navigation_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_TECS_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_InertialNav_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_WPNav_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_AttitudeControl_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Motors_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Landing_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_PID_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Soaring_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_LTM_Telem_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Devo_Telem_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_AutoTune_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_PrecLand_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_IRLock_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Quicktune_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_DroneCAN_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_HAL_SITL_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_CSVReader_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_SITL_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AP_Scripting_ArduPlane) #stripped2
target_link_libraries(ArduPlane_libs objs_AC_CustomControl_ArduPlane) #stripped2

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

add_executable(bin_arduplane ${bin_arduplane_SOURCES})
target_compile_definitions(bin_arduplane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2


target_link_libraries(bin_arduplane ArduPlane_libs) #clean

