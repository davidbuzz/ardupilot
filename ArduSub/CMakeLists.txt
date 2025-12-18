
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

add_library(objs_AP_AHRS_ArduSub ${objs_AP_AHRS_ArduSub_SOURCES})
target_compile_definitions(objs_AP_AHRS_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_AHRS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_ArduSub mavlink) #clean
target_link_libraries(objs_AP_AHRS_ArduSub dronecan) #clean
target_link_libraries(objs_AP_AHRS_ArduSub littlefs) #clean

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

add_library(objs_AP_Airspeed_ArduSub ${objs_AP_Airspeed_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Airspeed_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Airspeed_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Airspeed_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Airspeed_ArduSub littlefs) #clean

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

add_library(objs_AP_Baro_ArduSub ${objs_AP_Baro_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Baro_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Baro_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Baro_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Baro_ArduSub littlefs) #clean

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

add_library(objs_AP_BattMonitor_ArduSub ${objs_AP_BattMonitor_ArduSub_SOURCES})
target_compile_definitions(objs_AP_BattMonitor_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_BattMonitor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_ArduSub mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_ArduSub dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_ArduSub littlefs) #clean

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

add_library(objs_AP_BoardConfig_ArduSub ${objs_AP_BoardConfig_ArduSub_SOURCES})
target_compile_definitions(objs_AP_BoardConfig_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_BoardConfig_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_ArduSub mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_ArduSub dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_ArduSub littlefs) #clean

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

add_library(objs_AP_HAL_ArduSub ${objs_AP_HAL_ArduSub_SOURCES})
target_compile_definitions(objs_AP_HAL_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_HAL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_ArduSub mavlink) #clean
target_link_libraries(objs_AP_HAL_ArduSub dronecan) #clean
target_link_libraries(objs_AP_HAL_ArduSub littlefs) #clean

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

add_library(objs_AP_InertialSensor_ArduSub ${objs_AP_InertialSensor_ArduSub_SOURCES})
target_compile_definitions(objs_AP_InertialSensor_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_InertialSensor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_ArduSub mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_ArduSub dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_ArduSub littlefs) #clean

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

add_library(objs_AP_Math_ArduSub ${objs_AP_Math_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Math_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Math_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Math_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Math_ArduSub littlefs) #clean

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

add_library(objs_AP_Mission_ArduSub ${objs_AP_Mission_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Mission_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Mission_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Mission_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Mission_ArduSub littlefs) #clean

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

add_library(objs_AP_DAL_ArduSub ${objs_AP_DAL_ArduSub_SOURCES})
target_compile_definitions(objs_AP_DAL_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_DAL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_ArduSub mavlink) #clean
target_link_libraries(objs_AP_DAL_ArduSub dronecan) #clean
target_link_libraries(objs_AP_DAL_ArduSub littlefs) #clean

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

add_library(objs_AP_NavEKF2_ArduSub ${objs_AP_NavEKF2_ArduSub_SOURCES})
target_compile_definitions(objs_AP_NavEKF2_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_NavEKF2_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_ArduSub mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_ArduSub dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_ArduSub littlefs) #clean

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

add_library(objs_AP_NavEKF3_ArduSub ${objs_AP_NavEKF3_ArduSub_SOURCES})
target_compile_definitions(objs_AP_NavEKF3_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_NavEKF3_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_ArduSub mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_ArduSub dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_ArduSub littlefs) #clean

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

add_library(objs_AP_Rally_ArduSub ${objs_AP_Rally_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Rally_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Rally_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Rally_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Rally_ArduSub littlefs) #clean

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

add_library(objs_AP_Scheduler_ArduSub ${objs_AP_Scheduler_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Scheduler_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Scheduler_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Scheduler_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Scheduler_ArduSub littlefs) #clean

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

add_library(objs_AP_Terrain_ArduSub ${objs_AP_Terrain_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Terrain_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Terrain_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Terrain_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Terrain_ArduSub littlefs) #clean

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

add_library(objs_AP_Vehicle_ArduSub ${objs_AP_Vehicle_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Vehicle_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Vehicle_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Vehicle_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Vehicle_ArduSub littlefs) #clean

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

add_library(objs_AP_Logger_ArduSub ${objs_AP_Logger_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Logger_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Logger_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Logger_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Logger_ArduSub littlefs) #clean

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

add_library(objs_Filter_ArduSub ${objs_Filter_ArduSub_SOURCES})
target_compile_definitions(objs_Filter_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_Filter_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_ArduSub mavlink) #clean
target_link_libraries(objs_Filter_ArduSub dronecan) #clean
target_link_libraries(objs_Filter_ArduSub littlefs) #clean

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

add_library(objs_GCS_MAVLink_ArduSub ${objs_GCS_MAVLink_ArduSub_SOURCES})
target_compile_definitions(objs_GCS_MAVLink_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_GCS_MAVLink_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_ArduSub mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_ArduSub dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_ArduSub littlefs) #clean

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

add_library(objs_RC_Channel_ArduSub ${objs_RC_Channel_ArduSub_SOURCES})
target_compile_definitions(objs_RC_Channel_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_RC_Channel_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_ArduSub mavlink) #clean
target_link_libraries(objs_RC_Channel_ArduSub dronecan) #clean
target_link_libraries(objs_RC_Channel_ArduSub littlefs) #clean

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

add_library(objs_StorageManager_ArduSub ${objs_StorageManager_ArduSub_SOURCES})
target_compile_definitions(objs_StorageManager_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_StorageManager_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_ArduSub mavlink) #clean
target_link_libraries(objs_StorageManager_ArduSub dronecan) #clean
target_link_libraries(objs_StorageManager_ArduSub littlefs) #clean

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

add_library(objs_AP_Frsky_Telem_ArduSub ${objs_AP_Frsky_Telem_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Frsky_Telem_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Frsky_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduSub littlefs) #clean

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

add_library(objs_AP_IBus_Telem_ArduSub ${objs_AP_IBus_Telem_ArduSub_SOURCES})
target_compile_definitions(objs_AP_IBus_Telem_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_IBus_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_ArduSub mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduSub dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduSub littlefs) #clean

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

add_library(objs_AP_Relay_ArduSub ${objs_AP_Relay_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Relay_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Relay_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Relay_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Relay_ArduSub littlefs) #clean

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

add_library(objs_AP_RCProtocol_ArduSub ${objs_AP_RCProtocol_ArduSub_SOURCES})
target_compile_definitions(objs_AP_RCProtocol_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_RCProtocol_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_ArduSub mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_ArduSub dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_ArduSub littlefs) #clean

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

add_library(objs_AP_BLHeli_ArduSub ${objs_AP_BLHeli_ArduSub_SOURCES})
target_compile_definitions(objs_AP_BLHeli_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_BLHeli_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_ArduSub mavlink) #clean
target_link_libraries(objs_AP_BLHeli_ArduSub dronecan) #clean
target_link_libraries(objs_AP_BLHeli_ArduSub littlefs) #clean

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

add_library(objs_AP_Proximity_ArduSub ${objs_AP_Proximity_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Proximity_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Proximity_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Proximity_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Proximity_ArduSub littlefs) #clean

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

add_library(objs_AC_Fence_ArduSub ${objs_AC_Fence_ArduSub_SOURCES})
target_compile_definitions(objs_AC_Fence_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AC_Fence_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_ArduSub mavlink) #clean
target_link_libraries(objs_AC_Fence_ArduSub dronecan) #clean
target_link_libraries(objs_AC_Fence_ArduSub littlefs) #clean

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

add_library(objs_AC_Avoidance_ArduSub ${objs_AC_Avoidance_ArduSub_SOURCES})
target_compile_definitions(objs_AC_Avoidance_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AC_Avoidance_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_ArduSub mavlink) #clean
target_link_libraries(objs_AC_Avoidance_ArduSub dronecan) #clean
target_link_libraries(objs_AC_Avoidance_ArduSub littlefs) #clean

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

add_library(objs_AP_LandingGear_ArduSub ${objs_AP_LandingGear_ArduSub_SOURCES})
target_compile_definitions(objs_AP_LandingGear_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_LandingGear_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_ArduSub mavlink) #clean
target_link_libraries(objs_AP_LandingGear_ArduSub dronecan) #clean
target_link_libraries(objs_AP_LandingGear_ArduSub littlefs) #clean

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

add_library(objs_AP_OSD_ArduSub ${objs_AP_OSD_ArduSub_SOURCES})
target_compile_definitions(objs_AP_OSD_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_OSD_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_ArduSub mavlink) #clean
target_link_libraries(objs_AP_OSD_ArduSub dronecan) #clean
target_link_libraries(objs_AP_OSD_ArduSub littlefs) #clean

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

add_library(objs_AP_Filesystem_ArduSub ${objs_AP_Filesystem_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Filesystem_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Filesystem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Filesystem_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Filesystem_ArduSub littlefs) #clean

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

add_library(objs_AP_ADSB_ArduSub ${objs_AP_ADSB_ArduSub_SOURCES})
target_compile_definitions(objs_AP_ADSB_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_ADSB_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_ArduSub mavlink) #clean
target_link_libraries(objs_AP_ADSB_ArduSub dronecan) #clean
target_link_libraries(objs_AP_ADSB_ArduSub littlefs) #clean

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

add_library(objs_AP_ESC_Telem_ArduSub ${objs_AP_ESC_Telem_ArduSub_SOURCES})
target_compile_definitions(objs_AP_ESC_Telem_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_ESC_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_ArduSub mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduSub dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduSub littlefs) #clean

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

add_library(objs_AP_GyroFFT_ArduSub ${objs_AP_GyroFFT_ArduSub_SOURCES})
target_compile_definitions(objs_AP_GyroFFT_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_GyroFFT_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_ArduSub mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_ArduSub dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_ArduSub littlefs) #clean

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

add_library(objs_AP_RCTelemetry_ArduSub ${objs_AP_RCTelemetry_ArduSub_SOURCES})
target_compile_definitions(objs_AP_RCTelemetry_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_RCTelemetry_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_ArduSub mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduSub dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduSub littlefs) #clean

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

add_library(objs_AP_ExternalAHRS_ArduSub ${objs_AP_ExternalAHRS_ArduSub_SOURCES})
target_compile_definitions(objs_AP_ExternalAHRS_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_ExternalAHRS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_ArduSub mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduSub dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduSub littlefs) #clean

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

add_library(objs_AP_TemperatureSensor_ArduSub ${objs_AP_TemperatureSensor_ArduSub_SOURCES})
target_compile_definitions(objs_AP_TemperatureSensor_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_TemperatureSensor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_ArduSub mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduSub dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduSub littlefs) #clean

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

add_library(objs_AP_AIS_ArduSub ${objs_AP_AIS_ArduSub_SOURCES})
target_compile_definitions(objs_AP_AIS_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_AIS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_ArduSub mavlink) #clean
target_link_libraries(objs_AP_AIS_ArduSub dronecan) #clean
target_link_libraries(objs_AP_AIS_ArduSub littlefs) #clean

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

add_library(objs_AP_Arming_ArduSub ${objs_AP_Arming_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Arming_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Arming_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Arming_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Arming_ArduSub littlefs) #clean

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

add_library(objs_AP_Follow_ArduSub ${objs_AP_Follow_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Follow_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Follow_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Follow_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Follow_ArduSub littlefs) #clean

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

add_library(objs_AC_AttitudeControl_ArduSub ${objs_AC_AttitudeControl_ArduSub_SOURCES})
target_compile_definitions(objs_AC_AttitudeControl_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AC_AttitudeControl_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_ArduSub mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduSub dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduSub littlefs) #clean

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

add_library(objs_AC_WPNav_ArduSub ${objs_AC_WPNav_ArduSub_SOURCES})
target_compile_definitions(objs_AC_WPNav_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AC_WPNav_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav_ArduSub mavlink) #clean
target_link_libraries(objs_AC_WPNav_ArduSub dronecan) #clean
target_link_libraries(objs_AC_WPNav_ArduSub littlefs) #clean

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

add_library(objs_AP_Motors_ArduSub ${objs_AP_Motors_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Motors_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Motors_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Motors_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Motors_ArduSub littlefs) #clean

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

add_library(objs_AP_HAL_SITL_ArduSub ${objs_AP_HAL_SITL_ArduSub_SOURCES})
target_compile_definitions(objs_AP_HAL_SITL_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_HAL_SITL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_ArduSub mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduSub dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduSub littlefs) #clean

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

add_library(objs_SITL_ArduSub ${objs_SITL_ArduSub_SOURCES})
target_compile_definitions(objs_SITL_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_SITL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_ArduSub mavlink) #clean
target_link_libraries(objs_SITL_ArduSub dronecan) #clean
target_link_libraries(objs_SITL_ArduSub littlefs) #clean

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

add_library(objs_AP_Scripting_ArduSub ${objs_AP_Scripting_ArduSub_SOURCES})
target_compile_definitions(objs_AP_Scripting_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2

set_target_properties(objs_AP_Scripting_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Scripting_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Scripting_ArduSub littlefs) #clean

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


target_link_libraries(ArduSub_libs libobjs_AP_AccelCal.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_ADC.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_AHRS.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Airspeed.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Baro.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_BattMonitor.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_BoardConfig.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Camera.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Common.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Compass.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Declination.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_GPS.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_GSOF.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_HAL.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_HAL_Empty.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_DDS.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_InertialSensor.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Math.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Mission.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_DAL.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_NavEKF.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_NavEKF2.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_NavEKF3.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Notify.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_OpticalFlow.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Param.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Rally.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_LightWareSerial.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RangeFinder.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Scheduler.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_SerialManager.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Terrain.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Vehicle.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_InternalError.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Logger.a) #stripped
target_link_libraries(ArduSub_libs libobjs_Filter.a) #stripped
target_link_libraries(ArduSub_libs libobjs_GCS_MAVLink.a) #stripped
target_link_libraries(ArduSub_libs libobjs_RC_Channel.a) #stripped
target_link_libraries(ArduSub_libs libobjs_SRV_Channel.a) #stripped
target_link_libraries(ArduSub_libs libobjs_StorageManager.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Tuning.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RPM.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RSSI.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Mount.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Module.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Button.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_ICEngine.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Networking.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Frsky_Telem.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_IBus_Telem.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_FlashStorage.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Relay.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_ServoRelayEvents.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Volz_Protocol.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_SBusOut.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_IOMCU.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Parachute.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RAMTRON.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RCProtocol.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Radio.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_TempCalibration.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_VisualOdom.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_BLHeli.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_ROMFS.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Proximity.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Gripper.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RTC.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AC_Sprayer.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AC_Fence.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AC_Avoidance.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_LandingGear.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RobotisServo.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_NMEA_Output.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_OSD.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Filesystem.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_ADSB.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AC_PID.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_SerialLED.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_EFI.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Hott_Telem.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_ESC_Telem.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Servo_Telem.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Stats.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_GyroFFT.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RCTelemetry.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Generator.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_MSP.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_OLC.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_WheelEncoder.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_ExternalAHRS.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_VideoTX.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_FETtecOneWire.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_TemperatureSensor.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Torqeedo.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_CustomRotations.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_AIS.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_OpenDroneID.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_CheckFirmware.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_ExternalControl.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_JSON.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Beacon.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Arming.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_RCMapper.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_MultiHeap.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Follow.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_CANManager.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_KDECAN.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_PiccoloCAN.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AC_AttitudeControl.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AC_WPNav.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_InertialNav.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_JSButton.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_LeakDetector.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Motors.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_DroneCAN.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_HAL_SITL.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_CSVReader.a) #stripped
target_link_libraries(ArduSub_libs libobjs_SITL.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AP_Scripting.a) #stripped
target_link_libraries(ArduSub_libs libobjs_AC_CustomControl.a) #stripped

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

add_executable(bin_ardusub ${bin_ardusub_SOURCES})
target_compile_definitions(bin_ardusub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2


target_link_libraries(bin_ardusub ArduSub_libs) #clean

