
set(objs_AP_AHRS_ArduCopter_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs_AP_AHRS_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_AHRS_ArduCopter_INCLUDES})

add_library(objs_AP_AHRS_ArduCopter ${objs_AP_AHRS_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_AHRS_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_AHRS_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_AHRS_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_AHRS_ArduCopter littlefs) #clean

set(objs_AP_Airspeed_ArduCopter_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs_AP_Airspeed_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Airspeed_ArduCopter_INCLUDES})

add_library(objs_AP_Airspeed_ArduCopter ${objs_AP_Airspeed_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Airspeed_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Airspeed_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Airspeed_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Airspeed_ArduCopter littlefs) #clean

set(objs_AP_Baro_ArduCopter_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs_AP_Baro_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Baro_ArduCopter_INCLUDES})

add_library(objs_AP_Baro_ArduCopter ${objs_AP_Baro_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Baro_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Baro_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Baro_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Baro_ArduCopter littlefs) #clean

set(objs_AP_BattMonitor_ArduCopter_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs_AP_BattMonitor_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BattMonitor_ArduCopter_INCLUDES})

add_library(objs_AP_BattMonitor_ArduCopter ${objs_AP_BattMonitor_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_BattMonitor_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_BattMonitor_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_ArduCopter littlefs) #clean

set(objs_AP_BoardConfig_ArduCopter_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs_AP_BoardConfig_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BoardConfig_ArduCopter_INCLUDES})

add_library(objs_AP_BoardConfig_ArduCopter ${objs_AP_BoardConfig_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_BoardConfig_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_BoardConfig_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_ArduCopter littlefs) #clean

set(objs_AP_HAL_ArduCopter_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs_AP_HAL_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_HAL_ArduCopter_INCLUDES})

add_library(objs_AP_HAL_ArduCopter ${objs_AP_HAL_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_HAL_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_HAL_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_HAL_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_HAL_ArduCopter littlefs) #clean

set(objs_AP_InertialSensor_ArduCopter_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs_AP_InertialSensor_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_InertialSensor_ArduCopter_INCLUDES})

add_library(objs_AP_InertialSensor_ArduCopter ${objs_AP_InertialSensor_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_InertialSensor_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_InertialSensor_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_ArduCopter littlefs) #clean

set(objs_AP_Math_ArduCopter_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs_AP_Math_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Math_ArduCopter_INCLUDES})

add_library(objs_AP_Math_ArduCopter ${objs_AP_Math_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Math_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Math_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Math_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Math_ArduCopter littlefs) #clean

set(objs_AP_Mission_ArduCopter_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs_AP_Mission_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Mission_ArduCopter_INCLUDES})

add_library(objs_AP_Mission_ArduCopter ${objs_AP_Mission_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Mission_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Mission_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Mission_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Mission_ArduCopter littlefs) #clean

set(objs_AP_DAL_ArduCopter_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs_AP_DAL_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_DAL_ArduCopter_INCLUDES})

add_library(objs_AP_DAL_ArduCopter ${objs_AP_DAL_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_DAL_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_DAL_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_DAL_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_DAL_ArduCopter littlefs) #clean

set(objs_AP_NavEKF2_ArduCopter_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs_AP_NavEKF2_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_NavEKF2_ArduCopter_INCLUDES})

add_library(objs_AP_NavEKF2_ArduCopter ${objs_AP_NavEKF2_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_NavEKF2_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_NavEKF2_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_ArduCopter littlefs) #clean

set(objs_AP_NavEKF3_ArduCopter_SOURCES
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

set(objs_AP_NavEKF3_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_NavEKF3_ArduCopter_INCLUDES})

add_library(objs_AP_NavEKF3_ArduCopter ${objs_AP_NavEKF3_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_NavEKF3_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_NavEKF3_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_ArduCopter littlefs) #clean

set(objs_AP_Rally_ArduCopter_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs_AP_Rally_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Rally_ArduCopter_INCLUDES})

add_library(objs_AP_Rally_ArduCopter ${objs_AP_Rally_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Rally_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Rally_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Rally_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Rally_ArduCopter littlefs) #clean

set(objs_AP_Scheduler_ArduCopter_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs_AP_Scheduler_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Scheduler_ArduCopter_INCLUDES})

add_library(objs_AP_Scheduler_ArduCopter ${objs_AP_Scheduler_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Scheduler_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Scheduler_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Scheduler_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Scheduler_ArduCopter littlefs) #clean

set(objs_AP_Terrain_ArduCopter_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs_AP_Terrain_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Terrain_ArduCopter_INCLUDES})

add_library(objs_AP_Terrain_ArduCopter ${objs_AP_Terrain_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Terrain_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Terrain_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Terrain_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Terrain_ArduCopter littlefs) #clean

set(objs_AP_Vehicle_ArduCopter_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs_AP_Vehicle_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Vehicle_ArduCopter_INCLUDES})

add_library(objs_AP_Vehicle_ArduCopter ${objs_AP_Vehicle_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Vehicle_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Vehicle_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Vehicle_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Vehicle_ArduCopter littlefs) #clean

set(objs_AP_Logger_ArduCopter_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs_AP_Logger_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Logger_ArduCopter_INCLUDES})

add_library(objs_AP_Logger_ArduCopter ${objs_AP_Logger_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Logger_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Logger_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Logger_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Logger_ArduCopter littlefs) #clean

set(objs_Filter_ArduCopter_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs_Filter_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_Filter_ArduCopter_INCLUDES})

add_library(objs_Filter_ArduCopter ${objs_Filter_ArduCopter_SOURCES})
target_compile_definitions(objs_Filter_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_Filter_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_ArduCopter mavlink) #clean
target_link_libraries(objs_Filter_ArduCopter dronecan) #clean
target_link_libraries(objs_Filter_ArduCopter littlefs) #clean

set(objs_GCS_MAVLink_ArduCopter_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs_GCS_MAVLink_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_GCS_MAVLink_ArduCopter_INCLUDES})

add_library(objs_GCS_MAVLink_ArduCopter ${objs_GCS_MAVLink_ArduCopter_SOURCES})
target_compile_definitions(objs_GCS_MAVLink_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_GCS_MAVLink_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_ArduCopter mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_ArduCopter dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_ArduCopter littlefs) #clean

set(objs_RC_Channel_ArduCopter_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs_RC_Channel_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_RC_Channel_ArduCopter_INCLUDES})

add_library(objs_RC_Channel_ArduCopter ${objs_RC_Channel_ArduCopter_SOURCES})
target_compile_definitions(objs_RC_Channel_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_RC_Channel_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_ArduCopter mavlink) #clean
target_link_libraries(objs_RC_Channel_ArduCopter dronecan) #clean
target_link_libraries(objs_RC_Channel_ArduCopter littlefs) #clean

set(objs_StorageManager_ArduCopter_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs_StorageManager_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_StorageManager_ArduCopter_INCLUDES})

add_library(objs_StorageManager_ArduCopter ${objs_StorageManager_ArduCopter_SOURCES})
target_compile_definitions(objs_StorageManager_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_StorageManager_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_ArduCopter mavlink) #clean
target_link_libraries(objs_StorageManager_ArduCopter dronecan) #clean
target_link_libraries(objs_StorageManager_ArduCopter littlefs) #clean

set(objs_AP_Frsky_Telem_ArduCopter_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs_AP_Frsky_Telem_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Frsky_Telem_ArduCopter_INCLUDES})

add_library(objs_AP_Frsky_Telem_ArduCopter ${objs_AP_Frsky_Telem_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Frsky_Telem_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Frsky_Telem_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduCopter littlefs) #clean

set(objs_AP_IBus_Telem_ArduCopter_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs_AP_IBus_Telem_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_IBus_Telem_ArduCopter_INCLUDES})

add_library(objs_AP_IBus_Telem_ArduCopter ${objs_AP_IBus_Telem_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_IBus_Telem_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_IBus_Telem_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduCopter littlefs) #clean

set(objs_AP_Relay_ArduCopter_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs_AP_Relay_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Relay_ArduCopter_INCLUDES})

add_library(objs_AP_Relay_ArduCopter ${objs_AP_Relay_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Relay_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Relay_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Relay_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Relay_ArduCopter littlefs) #clean

set(objs_AP_RCProtocol_ArduCopter_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs_AP_RCProtocol_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_RCProtocol_ArduCopter_INCLUDES})

add_library(objs_AP_RCProtocol_ArduCopter ${objs_AP_RCProtocol_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_RCProtocol_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_RCProtocol_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_ArduCopter littlefs) #clean

set(objs_AP_BLHeli_ArduCopter_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs_AP_BLHeli_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_BLHeli_ArduCopter_INCLUDES})

add_library(objs_AP_BLHeli_ArduCopter ${objs_AP_BLHeli_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_BLHeli_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_BLHeli_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_BLHeli_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_BLHeli_ArduCopter littlefs) #clean

set(objs_AP_Proximity_ArduCopter_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs_AP_Proximity_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Proximity_ArduCopter_INCLUDES})

add_library(objs_AP_Proximity_ArduCopter ${objs_AP_Proximity_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Proximity_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Proximity_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Proximity_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Proximity_ArduCopter littlefs) #clean

set(objs_AC_Fence_ArduCopter_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs_AC_Fence_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_Fence_ArduCopter_INCLUDES})

add_library(objs_AC_Fence_ArduCopter ${objs_AC_Fence_ArduCopter_SOURCES})
target_compile_definitions(objs_AC_Fence_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AC_Fence_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_Fence_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_Fence_ArduCopter littlefs) #clean

set(objs_AC_Avoidance_ArduCopter_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs_AC_Avoidance_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_Avoidance_ArduCopter_INCLUDES})

add_library(objs_AC_Avoidance_ArduCopter ${objs_AC_Avoidance_ArduCopter_SOURCES})
target_compile_definitions(objs_AC_Avoidance_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AC_Avoidance_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_Avoidance_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_Avoidance_ArduCopter littlefs) #clean

set(objs_AP_LandingGear_ArduCopter_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs_AP_LandingGear_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_LandingGear_ArduCopter_INCLUDES})

add_library(objs_AP_LandingGear_ArduCopter ${objs_AP_LandingGear_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_LandingGear_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_LandingGear_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_LandingGear_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_LandingGear_ArduCopter littlefs) #clean

set(objs_AP_OSD_ArduCopter_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs_AP_OSD_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_OSD_ArduCopter_INCLUDES})

add_library(objs_AP_OSD_ArduCopter ${objs_AP_OSD_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_OSD_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_OSD_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_OSD_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_OSD_ArduCopter littlefs) #clean

set(objs_AP_Filesystem_ArduCopter_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs_AP_Filesystem_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Filesystem_ArduCopter_INCLUDES})

add_library(objs_AP_Filesystem_ArduCopter ${objs_AP_Filesystem_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Filesystem_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Filesystem_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Filesystem_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Filesystem_ArduCopter littlefs) #clean

set(objs_AP_ADSB_ArduCopter_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs_AP_ADSB_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ADSB_ArduCopter_INCLUDES})

add_library(objs_AP_ADSB_ArduCopter ${objs_AP_ADSB_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_ADSB_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_ADSB_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_ADSB_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_ADSB_ArduCopter littlefs) #clean

set(objs_AP_ESC_Telem_ArduCopter_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs_AP_ESC_Telem_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ESC_Telem_ArduCopter_INCLUDES})

add_library(objs_AP_ESC_Telem_ArduCopter ${objs_AP_ESC_Telem_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_ESC_Telem_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_ESC_Telem_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduCopter littlefs) #clean

set(objs_AP_GyroFFT_ArduCopter_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs_AP_GyroFFT_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_GyroFFT_ArduCopter_INCLUDES})

add_library(objs_AP_GyroFFT_ArduCopter ${objs_AP_GyroFFT_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_GyroFFT_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_GyroFFT_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_ArduCopter littlefs) #clean

set(objs_AP_RCTelemetry_ArduCopter_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs_AP_RCTelemetry_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_RCTelemetry_ArduCopter_INCLUDES})

add_library(objs_AP_RCTelemetry_ArduCopter ${objs_AP_RCTelemetry_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_RCTelemetry_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_RCTelemetry_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduCopter littlefs) #clean

set(objs_AP_ExternalAHRS_ArduCopter_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs_AP_ExternalAHRS_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_ExternalAHRS_ArduCopter_INCLUDES})

add_library(objs_AP_ExternalAHRS_ArduCopter ${objs_AP_ExternalAHRS_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_ExternalAHRS_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_ExternalAHRS_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduCopter littlefs) #clean

set(objs_AP_TemperatureSensor_ArduCopter_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs_AP_TemperatureSensor_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_TemperatureSensor_ArduCopter_INCLUDES})

add_library(objs_AP_TemperatureSensor_ArduCopter ${objs_AP_TemperatureSensor_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_TemperatureSensor_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_TemperatureSensor_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduCopter littlefs) #clean

set(objs_AP_AIS_ArduCopter_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs_AP_AIS_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_AIS_ArduCopter_INCLUDES})

add_library(objs_AP_AIS_ArduCopter ${objs_AP_AIS_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_AIS_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_AIS_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_AIS_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_AIS_ArduCopter littlefs) #clean

set(objs_AP_Arming_ArduCopter_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs_AP_Arming_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Arming_ArduCopter_INCLUDES})

add_library(objs_AP_Arming_ArduCopter ${objs_AP_Arming_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Arming_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Arming_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Arming_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Arming_ArduCopter littlefs) #clean

set(objs_AP_Follow_ArduCopter_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs_AP_Follow_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Follow_ArduCopter_INCLUDES})

add_library(objs_AP_Follow_ArduCopter ${objs_AP_Follow_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Follow_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Follow_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Follow_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Follow_ArduCopter littlefs) #clean

set(objs_AC_AttitudeControl_ArduCopter_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs_AC_AttitudeControl_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_AttitudeControl_ArduCopter_INCLUDES})

add_library(objs_AC_AttitudeControl_ArduCopter ${objs_AC_AttitudeControl_ArduCopter_SOURCES})
target_compile_definitions(objs_AC_AttitudeControl_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AC_AttitudeControl_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduCopter littlefs) #clean

set(objs_AC_PrecLand_ArduCopter_SOURCES
    ../libraries/AC_PrecLand/AC_PrecLand.cpp)

set(objs_AC_PrecLand_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_PrecLand_ArduCopter_INCLUDES})

add_library(objs_AC_PrecLand_ArduCopter ${objs_AC_PrecLand_ArduCopter_SOURCES})
target_compile_definitions(objs_AC_PrecLand_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AC_PrecLand_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_PrecLand_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_PrecLand_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_PrecLand_ArduCopter littlefs) #clean

set(objs_AC_WPNav_ArduCopter_SOURCES
    ../libraries/AC_WPNav/AC_Loiter.cpp)

set(objs_AC_WPNav_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_WPNav_ArduCopter_INCLUDES})

add_library(objs_AC_WPNav_ArduCopter ${objs_AC_WPNav_ArduCopter_SOURCES})
target_compile_definitions(objs_AC_WPNav_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AC_WPNav_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_WPNav_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_WPNav_ArduCopter littlefs) #clean

set(objs_AP_Motors_ArduCopter_SOURCES
    ../libraries/AP_Motors/AP_MotorsMatrix.cpp
    ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    ../libraries/AP_Motors/AP_MotorsTri.cpp
    ../libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp)

set(objs_AP_Motors_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Motors_ArduCopter_INCLUDES})

add_library(objs_AP_Motors_ArduCopter ${objs_AP_Motors_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Motors_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Motors_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Motors_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Motors_ArduCopter littlefs) #clean

set(objs_AP_Avoidance_ArduCopter_SOURCES
    ../libraries/AP_Avoidance/AP_Avoidance.cpp)

set(objs_AP_Avoidance_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Avoidance_ArduCopter_INCLUDES})

add_library(objs_AP_Avoidance_ArduCopter ${objs_AP_Avoidance_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Avoidance_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Avoidance_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Avoidance_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Avoidance_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Avoidance_ArduCopter littlefs) #clean

set(objs_AC_AutoTune_ArduCopter_SOURCES
    ../libraries/AC_AutoTune/AC_AutoTune.cpp
    ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp)

set(objs_AC_AutoTune_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AC_AutoTune_ArduCopter_INCLUDES})

add_library(objs_AC_AutoTune_ArduCopter ${objs_AC_AutoTune_ArduCopter_SOURCES})
target_compile_definitions(objs_AC_AutoTune_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AC_AutoTune_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AutoTune_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_AutoTune_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_AutoTune_ArduCopter littlefs) #clean

set(objs_AP_HAL_SITL_ArduCopter_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs_AP_HAL_SITL_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_HAL_SITL_ArduCopter_INCLUDES})

add_library(objs_AP_HAL_SITL_ArduCopter ${objs_AP_HAL_SITL_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_HAL_SITL_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_HAL_SITL_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduCopter littlefs) #clean

set(objs_SITL_ArduCopter_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs_SITL_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_SITL_ArduCopter_INCLUDES})

add_library(objs_SITL_ArduCopter ${objs_SITL_ArduCopter_SOURCES})
target_compile_definitions(objs_SITL_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_SITL_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_ArduCopter mavlink) #clean
target_link_libraries(objs_SITL_ArduCopter dronecan) #clean
target_link_libraries(objs_SITL_ArduCopter littlefs) #clean

set(objs_AP_Scripting_ArduCopter_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs_AP_Scripting_ArduCopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${objs_AP_Scripting_ArduCopter_INCLUDES})

add_library(objs_AP_Scripting_ArduCopter ${objs_AP_Scripting_ArduCopter_SOURCES})
target_compile_definitions(objs_AP_Scripting_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2

set_target_properties(objs_AP_Scripting_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Scripting_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Scripting_ArduCopter littlefs) #clean

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

set_target_properties(ArduCopter_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(ArduCopter_libs libobjs_AP_AccelCal.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_ADC.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_AHRS.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Airspeed.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Baro.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_BattMonitor.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_BoardConfig.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Camera.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Common.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Compass.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Declination.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_GPS.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_GSOF.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_HAL.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_HAL_Empty.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_DDS.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_InertialSensor.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Math.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Mission.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_DAL.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_NavEKF.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_NavEKF2.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_NavEKF3.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Notify.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_OpticalFlow.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Param.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Rally.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_LightWareSerial.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RangeFinder.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Scheduler.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_SerialManager.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Terrain.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Vehicle.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_InternalError.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Logger.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_Filter.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_GCS_MAVLink.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_RC_Channel.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_SRV_Channel.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_StorageManager.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Tuning.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RPM.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RSSI.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Mount.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Module.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Button.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_ICEngine.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Networking.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Frsky_Telem.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_IBus_Telem.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_FlashStorage.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Relay.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_ServoRelayEvents.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Volz_Protocol.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_SBusOut.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_IOMCU.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Parachute.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RAMTRON.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RCProtocol.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Radio.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_TempCalibration.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_VisualOdom.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_BLHeli.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_ROMFS.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Proximity.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Gripper.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RTC.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_Sprayer.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_Fence.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_Avoidance.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_LandingGear.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RobotisServo.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_NMEA_Output.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_OSD.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Filesystem.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_ADSB.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_PID.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_SerialLED.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_EFI.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Hott_Telem.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_ESC_Telem.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Servo_Telem.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Stats.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_GyroFFT.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RCTelemetry.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Generator.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_MSP.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_OLC.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_WheelEncoder.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_ExternalAHRS.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_VideoTX.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_FETtecOneWire.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_TemperatureSensor.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Torqeedo.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_CustomRotations.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_AIS.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_OpenDroneID.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_CheckFirmware.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_ExternalControl.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_JSON.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Beacon.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Arming.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_RCMapper.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_MultiHeap.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Follow.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_CANManager.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_KDECAN.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_PiccoloCAN.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_AttitudeControl.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_InputManager.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_PrecLand.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_Autorotation.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_WPNav.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_IRLock.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Motors.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Avoidance.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_AdvancedFailsafe.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_SmartRTL.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Winch.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_LTM_Telem.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Devo_Telem.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_AutoTune.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_SurfaceDistance.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_DroneCAN.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_HAL_SITL.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_CSVReader.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_SITL.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AP_Scripting.a) #stripped
target_link_libraries(ArduCopter_libs libobjs_AC_CustomControl.a) #stripped

set(bin_arducopter_SOURCES
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

set(bin_arducopter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${bin_arducopter_INCLUDES})

add_executable(bin_arducopter ${bin_arducopter_SOURCES})
target_compile_definitions(bin_arducopter PRIVATE -DFRAME_CONFIG=MULTICOPTER_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2


target_link_libraries(bin_arducopter ArduCopter_libs) #clean

set(bin_arducopter-heli_SOURCES
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

set(bin_arducopter-heli_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

include_directories(${bin_arducopter-heli_INCLUDES})

add_executable(bin_arducopter-heli ${bin_arducopter-heli_SOURCES})
target_compile_definitions(bin_arducopter-heli PRIVATE -DFRAME_CONFIG=HELI_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2


target_link_libraries(bin_arducopter-heli ArduCopter_libs) #clean

