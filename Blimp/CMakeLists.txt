
set(objs_AP_AHRS_Blimp_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs_AP_AHRS_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_AHRS_Blimp_INCLUDES})

add_library(objs_AP_AHRS_Blimp ${objs_AP_AHRS_Blimp_SOURCES})
target_compile_definitions(objs_AP_AHRS_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_AHRS_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_Blimp mavlink) #clean
target_link_libraries(objs_AP_AHRS_Blimp dronecan) #clean
target_link_libraries(objs_AP_AHRS_Blimp littlefs) #clean

set(objs_AP_Airspeed_Blimp_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs_AP_Airspeed_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Airspeed_Blimp_INCLUDES})

add_library(objs_AP_Airspeed_Blimp ${objs_AP_Airspeed_Blimp_SOURCES})
target_compile_definitions(objs_AP_Airspeed_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Airspeed_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_Blimp mavlink) #clean
target_link_libraries(objs_AP_Airspeed_Blimp dronecan) #clean
target_link_libraries(objs_AP_Airspeed_Blimp littlefs) #clean

set(objs_AP_Baro_Blimp_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs_AP_Baro_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Baro_Blimp_INCLUDES})

add_library(objs_AP_Baro_Blimp ${objs_AP_Baro_Blimp_SOURCES})
target_compile_definitions(objs_AP_Baro_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Baro_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_Blimp mavlink) #clean
target_link_libraries(objs_AP_Baro_Blimp dronecan) #clean
target_link_libraries(objs_AP_Baro_Blimp littlefs) #clean

set(objs_AP_BattMonitor_Blimp_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs_AP_BattMonitor_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BattMonitor_Blimp_INCLUDES})

add_library(objs_AP_BattMonitor_Blimp ${objs_AP_BattMonitor_Blimp_SOURCES})
target_compile_definitions(objs_AP_BattMonitor_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_BattMonitor_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_Blimp mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_Blimp dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_Blimp littlefs) #clean

set(objs_AP_BoardConfig_Blimp_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs_AP_BoardConfig_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BoardConfig_Blimp_INCLUDES})

add_library(objs_AP_BoardConfig_Blimp ${objs_AP_BoardConfig_Blimp_SOURCES})
target_compile_definitions(objs_AP_BoardConfig_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_BoardConfig_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_Blimp mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_Blimp dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_Blimp littlefs) #clean

set(objs_AP_HAL_Blimp_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs_AP_HAL_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_HAL_Blimp_INCLUDES})

add_library(objs_AP_HAL_Blimp ${objs_AP_HAL_Blimp_SOURCES})
target_compile_definitions(objs_AP_HAL_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_HAL_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_Blimp mavlink) #clean
target_link_libraries(objs_AP_HAL_Blimp dronecan) #clean
target_link_libraries(objs_AP_HAL_Blimp littlefs) #clean

set(objs_AP_InertialSensor_Blimp_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs_AP_InertialSensor_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_InertialSensor_Blimp_INCLUDES})

add_library(objs_AP_InertialSensor_Blimp ${objs_AP_InertialSensor_Blimp_SOURCES})
target_compile_definitions(objs_AP_InertialSensor_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_InertialSensor_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_Blimp mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_Blimp dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_Blimp littlefs) #clean

set(objs_AP_Math_Blimp_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs_AP_Math_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Math_Blimp_INCLUDES})

add_library(objs_AP_Math_Blimp ${objs_AP_Math_Blimp_SOURCES})
target_compile_definitions(objs_AP_Math_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Math_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_Blimp mavlink) #clean
target_link_libraries(objs_AP_Math_Blimp dronecan) #clean
target_link_libraries(objs_AP_Math_Blimp littlefs) #clean

set(objs_AP_Mission_Blimp_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs_AP_Mission_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Mission_Blimp_INCLUDES})

add_library(objs_AP_Mission_Blimp ${objs_AP_Mission_Blimp_SOURCES})
target_compile_definitions(objs_AP_Mission_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Mission_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_Blimp mavlink) #clean
target_link_libraries(objs_AP_Mission_Blimp dronecan) #clean
target_link_libraries(objs_AP_Mission_Blimp littlefs) #clean

set(objs_AP_DAL_Blimp_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs_AP_DAL_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_DAL_Blimp_INCLUDES})

add_library(objs_AP_DAL_Blimp ${objs_AP_DAL_Blimp_SOURCES})
target_compile_definitions(objs_AP_DAL_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_DAL_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_Blimp mavlink) #clean
target_link_libraries(objs_AP_DAL_Blimp dronecan) #clean
target_link_libraries(objs_AP_DAL_Blimp littlefs) #clean

set(objs_AP_NavEKF2_Blimp_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs_AP_NavEKF2_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_NavEKF2_Blimp_INCLUDES})

add_library(objs_AP_NavEKF2_Blimp ${objs_AP_NavEKF2_Blimp_SOURCES})
target_compile_definitions(objs_AP_NavEKF2_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_NavEKF2_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_Blimp mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_Blimp dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_Blimp littlefs) #clean

set(objs_AP_NavEKF3_Blimp_SOURCES
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

set(objs_AP_NavEKF3_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_NavEKF3_Blimp_INCLUDES})

add_library(objs_AP_NavEKF3_Blimp ${objs_AP_NavEKF3_Blimp_SOURCES})
target_compile_definitions(objs_AP_NavEKF3_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_NavEKF3_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_Blimp mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_Blimp dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_Blimp littlefs) #clean

set(objs_AP_Rally_Blimp_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs_AP_Rally_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Rally_Blimp_INCLUDES})

add_library(objs_AP_Rally_Blimp ${objs_AP_Rally_Blimp_SOURCES})
target_compile_definitions(objs_AP_Rally_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Rally_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_Blimp mavlink) #clean
target_link_libraries(objs_AP_Rally_Blimp dronecan) #clean
target_link_libraries(objs_AP_Rally_Blimp littlefs) #clean

set(objs_AP_Scheduler_Blimp_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs_AP_Scheduler_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Scheduler_Blimp_INCLUDES})

add_library(objs_AP_Scheduler_Blimp ${objs_AP_Scheduler_Blimp_SOURCES})
target_compile_definitions(objs_AP_Scheduler_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Scheduler_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_Blimp mavlink) #clean
target_link_libraries(objs_AP_Scheduler_Blimp dronecan) #clean
target_link_libraries(objs_AP_Scheduler_Blimp littlefs) #clean

set(objs_AP_Terrain_Blimp_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs_AP_Terrain_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Terrain_Blimp_INCLUDES})

add_library(objs_AP_Terrain_Blimp ${objs_AP_Terrain_Blimp_SOURCES})
target_compile_definitions(objs_AP_Terrain_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Terrain_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_Blimp mavlink) #clean
target_link_libraries(objs_AP_Terrain_Blimp dronecan) #clean
target_link_libraries(objs_AP_Terrain_Blimp littlefs) #clean

set(objs_AP_Vehicle_Blimp_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs_AP_Vehicle_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Vehicle_Blimp_INCLUDES})

add_library(objs_AP_Vehicle_Blimp ${objs_AP_Vehicle_Blimp_SOURCES})
target_compile_definitions(objs_AP_Vehicle_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Vehicle_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_Blimp mavlink) #clean
target_link_libraries(objs_AP_Vehicle_Blimp dronecan) #clean
target_link_libraries(objs_AP_Vehicle_Blimp littlefs) #clean

set(objs_AP_Logger_Blimp_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs_AP_Logger_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Logger_Blimp_INCLUDES})

add_library(objs_AP_Logger_Blimp ${objs_AP_Logger_Blimp_SOURCES})
target_compile_definitions(objs_AP_Logger_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Logger_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_Blimp mavlink) #clean
target_link_libraries(objs_AP_Logger_Blimp dronecan) #clean
target_link_libraries(objs_AP_Logger_Blimp littlefs) #clean

set(objs_Filter_Blimp_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs_Filter_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_Filter_Blimp_INCLUDES})

add_library(objs_Filter_Blimp ${objs_Filter_Blimp_SOURCES})
target_compile_definitions(objs_Filter_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_Filter_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_Blimp mavlink) #clean
target_link_libraries(objs_Filter_Blimp dronecan) #clean
target_link_libraries(objs_Filter_Blimp littlefs) #clean

set(objs_GCS_MAVLink_Blimp_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs_GCS_MAVLink_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_GCS_MAVLink_Blimp_INCLUDES})

add_library(objs_GCS_MAVLink_Blimp ${objs_GCS_MAVLink_Blimp_SOURCES})
target_compile_definitions(objs_GCS_MAVLink_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_GCS_MAVLink_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_Blimp mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_Blimp dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_Blimp littlefs) #clean

set(objs_RC_Channel_Blimp_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs_RC_Channel_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_RC_Channel_Blimp_INCLUDES})

add_library(objs_RC_Channel_Blimp ${objs_RC_Channel_Blimp_SOURCES})
target_compile_definitions(objs_RC_Channel_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_RC_Channel_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_Blimp mavlink) #clean
target_link_libraries(objs_RC_Channel_Blimp dronecan) #clean
target_link_libraries(objs_RC_Channel_Blimp littlefs) #clean

set(objs_StorageManager_Blimp_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs_StorageManager_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_StorageManager_Blimp_INCLUDES})

add_library(objs_StorageManager_Blimp ${objs_StorageManager_Blimp_SOURCES})
target_compile_definitions(objs_StorageManager_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_StorageManager_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_Blimp mavlink) #clean
target_link_libraries(objs_StorageManager_Blimp dronecan) #clean
target_link_libraries(objs_StorageManager_Blimp littlefs) #clean

set(objs_AP_Frsky_Telem_Blimp_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs_AP_Frsky_Telem_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Frsky_Telem_Blimp_INCLUDES})

add_library(objs_AP_Frsky_Telem_Blimp ${objs_AP_Frsky_Telem_Blimp_SOURCES})
target_compile_definitions(objs_AP_Frsky_Telem_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Frsky_Telem_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_Blimp mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_Blimp dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_Blimp littlefs) #clean

set(objs_AP_IBus_Telem_Blimp_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs_AP_IBus_Telem_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_IBus_Telem_Blimp_INCLUDES})

add_library(objs_AP_IBus_Telem_Blimp ${objs_AP_IBus_Telem_Blimp_SOURCES})
target_compile_definitions(objs_AP_IBus_Telem_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_IBus_Telem_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_Blimp mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_Blimp dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_Blimp littlefs) #clean

set(objs_AP_Relay_Blimp_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs_AP_Relay_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Relay_Blimp_INCLUDES})

add_library(objs_AP_Relay_Blimp ${objs_AP_Relay_Blimp_SOURCES})
target_compile_definitions(objs_AP_Relay_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Relay_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_Blimp mavlink) #clean
target_link_libraries(objs_AP_Relay_Blimp dronecan) #clean
target_link_libraries(objs_AP_Relay_Blimp littlefs) #clean

set(objs_AP_RCProtocol_Blimp_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs_AP_RCProtocol_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_RCProtocol_Blimp_INCLUDES})

add_library(objs_AP_RCProtocol_Blimp ${objs_AP_RCProtocol_Blimp_SOURCES})
target_compile_definitions(objs_AP_RCProtocol_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_RCProtocol_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_Blimp mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_Blimp dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_Blimp littlefs) #clean

set(objs_AP_BLHeli_Blimp_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs_AP_BLHeli_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BLHeli_Blimp_INCLUDES})

add_library(objs_AP_BLHeli_Blimp ${objs_AP_BLHeli_Blimp_SOURCES})
target_compile_definitions(objs_AP_BLHeli_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_BLHeli_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_Blimp mavlink) #clean
target_link_libraries(objs_AP_BLHeli_Blimp dronecan) #clean
target_link_libraries(objs_AP_BLHeli_Blimp littlefs) #clean

set(objs_AP_Proximity_Blimp_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs_AP_Proximity_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Proximity_Blimp_INCLUDES})

add_library(objs_AP_Proximity_Blimp ${objs_AP_Proximity_Blimp_SOURCES})
target_compile_definitions(objs_AP_Proximity_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Proximity_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_Blimp mavlink) #clean
target_link_libraries(objs_AP_Proximity_Blimp dronecan) #clean
target_link_libraries(objs_AP_Proximity_Blimp littlefs) #clean

set(objs_AC_Fence_Blimp_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs_AC_Fence_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_Fence_Blimp_INCLUDES})

add_library(objs_AC_Fence_Blimp ${objs_AC_Fence_Blimp_SOURCES})
target_compile_definitions(objs_AC_Fence_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AC_Fence_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_Blimp mavlink) #clean
target_link_libraries(objs_AC_Fence_Blimp dronecan) #clean
target_link_libraries(objs_AC_Fence_Blimp littlefs) #clean

set(objs_AC_Avoidance_Blimp_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs_AC_Avoidance_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_Avoidance_Blimp_INCLUDES})

add_library(objs_AC_Avoidance_Blimp ${objs_AC_Avoidance_Blimp_SOURCES})
target_compile_definitions(objs_AC_Avoidance_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AC_Avoidance_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_Blimp mavlink) #clean
target_link_libraries(objs_AC_Avoidance_Blimp dronecan) #clean
target_link_libraries(objs_AC_Avoidance_Blimp littlefs) #clean

set(objs_AP_LandingGear_Blimp_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs_AP_LandingGear_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_LandingGear_Blimp_INCLUDES})

add_library(objs_AP_LandingGear_Blimp ${objs_AP_LandingGear_Blimp_SOURCES})
target_compile_definitions(objs_AP_LandingGear_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_LandingGear_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_Blimp mavlink) #clean
target_link_libraries(objs_AP_LandingGear_Blimp dronecan) #clean
target_link_libraries(objs_AP_LandingGear_Blimp littlefs) #clean

set(objs_AP_OSD_Blimp_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs_AP_OSD_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_OSD_Blimp_INCLUDES})

add_library(objs_AP_OSD_Blimp ${objs_AP_OSD_Blimp_SOURCES})
target_compile_definitions(objs_AP_OSD_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_OSD_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_Blimp mavlink) #clean
target_link_libraries(objs_AP_OSD_Blimp dronecan) #clean
target_link_libraries(objs_AP_OSD_Blimp littlefs) #clean

set(objs_AP_Filesystem_Blimp_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs_AP_Filesystem_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Filesystem_Blimp_INCLUDES})

add_library(objs_AP_Filesystem_Blimp ${objs_AP_Filesystem_Blimp_SOURCES})
target_compile_definitions(objs_AP_Filesystem_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Filesystem_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_Blimp mavlink) #clean
target_link_libraries(objs_AP_Filesystem_Blimp dronecan) #clean
target_link_libraries(objs_AP_Filesystem_Blimp littlefs) #clean

set(objs_AP_ADSB_Blimp_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs_AP_ADSB_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ADSB_Blimp_INCLUDES})

add_library(objs_AP_ADSB_Blimp ${objs_AP_ADSB_Blimp_SOURCES})
target_compile_definitions(objs_AP_ADSB_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_ADSB_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_Blimp mavlink) #clean
target_link_libraries(objs_AP_ADSB_Blimp dronecan) #clean
target_link_libraries(objs_AP_ADSB_Blimp littlefs) #clean

set(objs_AP_ESC_Telem_Blimp_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs_AP_ESC_Telem_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ESC_Telem_Blimp_INCLUDES})

add_library(objs_AP_ESC_Telem_Blimp ${objs_AP_ESC_Telem_Blimp_SOURCES})
target_compile_definitions(objs_AP_ESC_Telem_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_ESC_Telem_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_Blimp mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_Blimp dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_Blimp littlefs) #clean

set(objs_AP_GyroFFT_Blimp_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs_AP_GyroFFT_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_GyroFFT_Blimp_INCLUDES})

add_library(objs_AP_GyroFFT_Blimp ${objs_AP_GyroFFT_Blimp_SOURCES})
target_compile_definitions(objs_AP_GyroFFT_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_GyroFFT_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_Blimp mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_Blimp dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_Blimp littlefs) #clean

set(objs_AP_RCTelemetry_Blimp_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs_AP_RCTelemetry_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_RCTelemetry_Blimp_INCLUDES})

add_library(objs_AP_RCTelemetry_Blimp ${objs_AP_RCTelemetry_Blimp_SOURCES})
target_compile_definitions(objs_AP_RCTelemetry_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_RCTelemetry_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_Blimp mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_Blimp dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_Blimp littlefs) #clean

set(objs_AP_ExternalAHRS_Blimp_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs_AP_ExternalAHRS_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ExternalAHRS_Blimp_INCLUDES})

add_library(objs_AP_ExternalAHRS_Blimp ${objs_AP_ExternalAHRS_Blimp_SOURCES})
target_compile_definitions(objs_AP_ExternalAHRS_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_ExternalAHRS_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_Blimp mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_Blimp dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_Blimp littlefs) #clean

set(objs_AP_TemperatureSensor_Blimp_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs_AP_TemperatureSensor_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_TemperatureSensor_Blimp_INCLUDES})

add_library(objs_AP_TemperatureSensor_Blimp ${objs_AP_TemperatureSensor_Blimp_SOURCES})
target_compile_definitions(objs_AP_TemperatureSensor_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_TemperatureSensor_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_Blimp mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_Blimp dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_Blimp littlefs) #clean

set(objs_AP_AIS_Blimp_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs_AP_AIS_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_AIS_Blimp_INCLUDES})

add_library(objs_AP_AIS_Blimp ${objs_AP_AIS_Blimp_SOURCES})
target_compile_definitions(objs_AP_AIS_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_AIS_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_Blimp mavlink) #clean
target_link_libraries(objs_AP_AIS_Blimp dronecan) #clean
target_link_libraries(objs_AP_AIS_Blimp littlefs) #clean

set(objs_AP_Arming_Blimp_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs_AP_Arming_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Arming_Blimp_INCLUDES})

add_library(objs_AP_Arming_Blimp ${objs_AP_Arming_Blimp_SOURCES})
target_compile_definitions(objs_AP_Arming_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Arming_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_Blimp mavlink) #clean
target_link_libraries(objs_AP_Arming_Blimp dronecan) #clean
target_link_libraries(objs_AP_Arming_Blimp littlefs) #clean

set(objs_AP_Follow_Blimp_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs_AP_Follow_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Follow_Blimp_INCLUDES})

add_library(objs_AP_Follow_Blimp ${objs_AP_Follow_Blimp_SOURCES})
target_compile_definitions(objs_AP_Follow_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Follow_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_Blimp mavlink) #clean
target_link_libraries(objs_AP_Follow_Blimp dronecan) #clean
target_link_libraries(objs_AP_Follow_Blimp littlefs) #clean

set(objs_AP_Avoidance_Blimp_SOURCES
    ../libraries/AP_Avoidance/AP_Avoidance.cpp)

set(objs_AP_Avoidance_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Avoidance_Blimp_INCLUDES})

add_library(objs_AP_Avoidance_Blimp ${objs_AP_Avoidance_Blimp_SOURCES})
target_compile_definitions(objs_AP_Avoidance_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Avoidance_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Avoidance_Blimp mavlink) #clean
target_link_libraries(objs_AP_Avoidance_Blimp dronecan) #clean
target_link_libraries(objs_AP_Avoidance_Blimp littlefs) #clean

set(objs_AC_AttitudeControl_Blimp_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs_AC_AttitudeControl_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_AttitudeControl_Blimp_INCLUDES})

add_library(objs_AC_AttitudeControl_Blimp ${objs_AC_AttitudeControl_Blimp_SOURCES})
target_compile_definitions(objs_AC_AttitudeControl_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AC_AttitudeControl_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_Blimp mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_Blimp dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_Blimp littlefs) #clean

set(objs_AP_HAL_SITL_Blimp_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs_AP_HAL_SITL_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_HAL_SITL_Blimp_INCLUDES})

add_library(objs_AP_HAL_SITL_Blimp ${objs_AP_HAL_SITL_Blimp_SOURCES})
target_compile_definitions(objs_AP_HAL_SITL_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_HAL_SITL_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_Blimp mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_Blimp dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_Blimp littlefs) #clean

set(objs_SITL_Blimp_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs_SITL_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_SITL_Blimp_INCLUDES})

add_library(objs_SITL_Blimp ${objs_SITL_Blimp_SOURCES})
target_compile_definitions(objs_SITL_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_SITL_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_Blimp mavlink) #clean
target_link_libraries(objs_SITL_Blimp dronecan) #clean
target_link_libraries(objs_SITL_Blimp littlefs) #clean

set(objs_AP_Scripting_Blimp_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs_AP_Scripting_Blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Scripting_Blimp_INCLUDES})

add_library(objs_AP_Scripting_Blimp ${objs_AP_Scripting_Blimp_SOURCES})
target_compile_definitions(objs_AP_Scripting_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2

set_target_properties(objs_AP_Scripting_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_Blimp mavlink) #clean
target_link_libraries(objs_AP_Scripting_Blimp dronecan) #clean
target_link_libraries(objs_AP_Scripting_Blimp littlefs) #clean

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

set_target_properties(Blimp_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(Blimp_libs objs_AP_AccelCal_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_ADC_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_AHRS_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Airspeed_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Baro_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_BattMonitor_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_BoardConfig_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Camera_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Common_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Compass_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Declination_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_GPS_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_GSOF_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_HAL_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_HAL_Empty_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_DDS_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_InertialSensor_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Math_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Mission_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_DAL_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_NavEKF_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_NavEKF2_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_NavEKF3_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Notify_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_OpticalFlow_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Param_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Rally_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_LightWareSerial_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RangeFinder_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Scheduler_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_SerialManager_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Terrain_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Vehicle_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_InternalError_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Logger_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_Filter_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_GCS_MAVLink_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_RC_Channel_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_SRV_Channel_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_StorageManager_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Tuning_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RPM_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RSSI_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Mount_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Module_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Button_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_ICEngine_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Networking_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Frsky_Telem_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_IBus_Telem_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_FlashStorage_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Relay_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_ServoRelayEvents_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Volz_Protocol_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_SBusOut_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_IOMCU_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Parachute_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RAMTRON_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RCProtocol_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Radio_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_TempCalibration_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_VisualOdom_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_BLHeli_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_ROMFS_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Proximity_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Gripper_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RTC_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AC_Sprayer_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AC_Fence_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AC_Avoidance_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_LandingGear_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RobotisServo_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_NMEA_Output_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_OSD_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Filesystem_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_ADSB_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AC_PID_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_SerialLED_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_EFI_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Hott_Telem_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_ESC_Telem_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Servo_Telem_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Stats_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_GyroFFT_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RCTelemetry_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Generator_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_MSP_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_OLC_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_WheelEncoder_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_ExternalAHRS_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_VideoTX_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_FETtecOneWire_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_TemperatureSensor_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Torqeedo_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_CustomRotations_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_AIS_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_OpenDroneID_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_CheckFirmware_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_ExternalControl_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_JSON_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Beacon_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Arming_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_RCMapper_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_MultiHeap_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Follow_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_CANManager_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_KDECAN_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_PiccoloCAN_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AC_InputManager_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_InertialNav_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Avoidance_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_LTM_Telem_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Devo_Telem_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_AdvancedFailsafe_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AC_AttitudeControl_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_DroneCAN_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_HAL_SITL_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_CSVReader_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_SITL_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AP_Scripting_Blimp) #stripped2
target_link_libraries(Blimp_libs objs_AC_CustomControl_Blimp) #stripped2

set(bin_blimp_SOURCES
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

set(bin_blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${bin_blimp_INCLUDES})

add_executable(bin_blimp ${bin_blimp_SOURCES})
target_compile_definitions(bin_blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2


target_link_libraries(bin_blimp Blimp_libs) #clean

