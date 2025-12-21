cmake_minimum_required (VERSION 3.13.0)
project (ArduPilot)

include_directories(.)
include_directories(..)
include_directories(../..)
include_directories(../../..)
include_directories(
${CMAKE_CURRENT_SOURCE_DIR}
${CMAKE_CURRENT_SOURCE_DIR}/AP_Common
${CMAKE_CURRENT_SOURCE_DIR}/AP_HAL
${CMAKE_CURRENT_SOURCE_DIR}/AP_HAL/board
${CMAKE_CURRENT_SOURCE_DIR}/libraries
"${CMAKE_BINARY_DIR}/sitl/libraries"
"${CMAKE_BINARY_DIR}/sitl/libraries/GCS_MAVLink"
${CMAKE_CURRENT_SOURCE_DIR}/modules/DroneCAN/libcanard
${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_DroneCAN/canard 
"${CMAKE_BINARY_DIR}/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include" 
)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_definitions(-DAP_BARO_PROBE_EXTERNAL_I2C_BUSES=1 -DAP_CUSTOMCONTROL_ENABLED=1 -DAP_DDS_ENABLED=0 -DAP_SCRIPTING_CHECKS=1 -DCANARD_ALLOCATE_SEM=1 -DCANARD_ENABLE_ASSERTS=1 -DCANARD_ENABLE_CANFD=1 -DCANARD_ENABLE_DEADLINE=1 -DCANARD_IFACE_ALL=3 -DCANARD_MULTI_IFACE=1 -DCONFIG_HAL_BOARD=HAL_BOARD_SITL -DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE -DDRONECAN_CXX_WRAPPERS=1 -DENABLE_ONVIF=0 -DLUA_32BITS=1 -DUSE_USER_HELPERS=1) #1

set(CMAKE_C_FLAGS "-ffunction-sections -fdata-sections -fsigned-char -Wall -Wextra -Werror=format -Wpointer-arith -Wcast-align -Wno-missing-field-initializers -Wno-unused-parameter -Wno-redundant-decls -Wno-unknown-pragmas -Wno-trigraphs -Werror=return-type -Werror=unused-result -Werror=unused-variable -Werror=narrowing -Werror=attributes -Werror=overflow -Werror=parentheses -Werror=format-extra-args -Werror=ignored-qualifiers -Werror=undef -DARDUPILOT_BUILD -Wno-format-contains-nul -fsingle-precision-constant -Werror=use-after-free -DCANARD_64_BIT=1 -MMD -ffile-prefix-map=../..=../..")
set(CMAKE_CXX_FLAGS "-std=gnu++11 -fdata-sections -ffunction-sections -fno-exceptions -fsigned-char -Wall -Wextra -Wpointer-arith -Wno-unused-parameter -Wno-missing-field-initializers -Wno-redundant-decls -Wno-unknown-pragmas -Wno-expansion-to-defined -Werror=reorder -Werror=cast-align -Werror=attributes -Werror=format-security -Werror=format-extra-args -Werror=enum-compare -Werror=format -Werror=array-bounds -Werror=uninitialized -Werror=init-self -Werror=narrowing -Werror=return-type -Werror=switch -Werror=sign-compare -Werror=type-limits -Werror=undef -Werror=unused-result -Werror=unused-value -Werror=unused-variable -Werror=delete-non-virtual-dtor -Wfatal-errors -Wno-trigraphs -Werror=parentheses -DARDUPILOT_BUILD -Wuninitialized -Warray-bounds -Wno-format-contains-nul -Werror=unused-but-set-variable -fsingle-precision-constant -Wno-psabi -Werror=suggest-override -Werror=implicit-fallthrough -Werror=maybe-uninitialized -Werror=duplicated-cond -Werror=sizeof-pointer-div -Werror=use-after-free -D__AP_LINE__=__LINE__ -DAP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED=HAL_GCS_ENABLED -DAC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT=HAL_GCS_ENABLED -DHAL_NAVEKF2_AVAILABLE=1 -DCANARD_64_BIT=1 -Werror=float-equal -Werror=missing-declarations -DAP_NETWORKING_ENABLED=1 -O3 -MMD -ffile-prefix-map=../..=../.. -DHAL_HAVE_AP_ROMFS_EMBEDDED_H  -D_AP_CONFIG_H_=1 -DWAF_BUILD=1 -D__STDC_FORMAT_MACROS=1 -DAP_SIM_ENABLED=1 -DHAL_WITH_SPI=1 -DHAL_WITH_RAMTRON=1 -DAP_OPENDRONEID_ENABLED=1 -DAP_SIGNED_FIRMWARE=0 -DAP_NOTIFY_LP5562_BUS=2 -DAP_NOTIFY_LP5562_ADDR=48 -DHAL_NUM_CAN_IFACES=2 -DHAL_CAN_WITH_SOCKETCAN=1 -DHAVE_FEENABLEEXCEPT=1 -DHAVE_CMATH_ISFINITE=1 -DHAVE_CMATH_ISINF=1 -DHAVE_CMATH_ISNAN=1 -DNEED_CMATH_ISFINITE_STD_NAMESPACE=1 -DNEED_CMATH_ISINF_STD_NAMESPACE=1 -DNEED_CMATH_ISNAN_STD_NAMESPACE=1 -DHAVE_ENDIAN_H=1 -DHAVE_BYTESWAP_H=1 -DHAVE_MEMRCHR=1 -D_GNU_SOURCE=1")

#------------------------------------------------

set(littlefs_SOURCES
    modules/littlefs/lfs.c
    modules/littlefs/lfs_util.c
    modules/littlefs/bd/lfs_filebd.c)

set(littlefs_INCLUDES
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include)

add_library(littlefs ${littlefs_SOURCES}) #2
target_compile_definitions(littlefs PRIVATE -DLFS_NO_DEBUG -DLFS_NO_WARN -DLFS_NO_ERROR -DLFS_NO_ASSERT) #2
target_include_directories(littlefs PRIVATE ${littlefs_INCLUDES})

set_target_properties(littlefs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

#------------------------------------------------

set(AC_AttitudeControl_SOURCES
    libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_TS.cpp
    libraries/AC_AttitudeControl/AC_CommandModel.cpp
    libraries/AC_AttitudeControl/AC_PosControl_Logging.cpp)

set(AC_AttitudeControl_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_AttitudeControl ${AC_AttitudeControl_SOURCES}) #2
target_include_directories(AC_AttitudeControl PRIVATE ${AC_AttitudeControl_INCLUDES})

set_target_properties(AC_AttitudeControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_AttitudeControl mavlink) #clean
target_link_libraries(AC_AttitudeControl dronecan) #clean
target_link_libraries(AC_AttitudeControl littlefs) #clean

#------------------------------------------------

set(AC_AutoTune_SOURCES
    libraries/AC_AutoTune/AC_AutoTune_FreqResp.cpp
    libraries/AC_AutoTune/AC_AutoTune_Heli.cpp)

set(AC_AutoTune_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_AutoTune ${AC_AutoTune_SOURCES}) #2
target_include_directories(AC_AutoTune PRIVATE ${AC_AutoTune_INCLUDES})

set_target_properties(AC_AutoTune PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_AutoTune mavlink) #clean
target_link_libraries(AC_AutoTune dronecan) #clean
target_link_libraries(AC_AutoTune littlefs) #clean

#------------------------------------------------

set(AC_Autorotation_SOURCES
    libraries/AC_Autorotation/AC_Autorotation.cpp
    libraries/AC_Autorotation/RSC_Autorotation.cpp)

set(AC_Autorotation_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_Autorotation ${AC_Autorotation_SOURCES}) #2
target_include_directories(AC_Autorotation PRIVATE ${AC_Autorotation_INCLUDES})

set_target_properties(AC_Autorotation PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_Autorotation mavlink) #clean
target_link_libraries(AC_Autorotation dronecan) #clean
target_link_libraries(AC_Autorotation littlefs) #clean

#------------------------------------------------

set(AC_Avoidance_SOURCES
    libraries/AC_Avoidance/AC_Avoidance_Logging.cpp
    libraries/AC_Avoidance/AP_OADijkstra.cpp
    libraries/AC_Avoidance/AP_OAPathPlanner.cpp
    libraries/AC_Avoidance/AP_OAVisGraph.cpp)

set(AC_Avoidance_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_Avoidance ${AC_Avoidance_SOURCES}) #2
target_include_directories(AC_Avoidance PRIVATE ${AC_Avoidance_INCLUDES})

set_target_properties(AC_Avoidance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_Avoidance mavlink) #clean
target_link_libraries(AC_Avoidance dronecan) #clean
target_link_libraries(AC_Avoidance littlefs) #clean

#------------------------------------------------

set(AC_CustomControl_SOURCES
    libraries/AC_CustomControl/AC_CustomControl.cpp
    libraries/AC_CustomControl/AC_CustomControl_Empty.cpp
    libraries/AC_CustomControl/AC_CustomControl_PID.cpp)

set(AC_CustomControl_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_CustomControl ${AC_CustomControl_SOURCES}) #2
target_include_directories(AC_CustomControl PRIVATE ${AC_CustomControl_INCLUDES})

set_target_properties(AC_CustomControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_CustomControl mavlink) #clean
target_link_libraries(AC_CustomControl dronecan) #clean
target_link_libraries(AC_CustomControl littlefs) #clean

#------------------------------------------------

set(AC_InputManager_SOURCES
    libraries/AC_InputManager/AC_InputManager.cpp
    libraries/AC_InputManager/AC_InputManager_Heli.cpp)

set(AC_InputManager_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_InputManager ${AC_InputManager_SOURCES}) #2
target_include_directories(AC_InputManager PRIVATE ${AC_InputManager_INCLUDES})

set_target_properties(AC_InputManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_InputManager mavlink) #clean
target_link_libraries(AC_InputManager dronecan) #clean
target_link_libraries(AC_InputManager littlefs) #clean

#------------------------------------------------

set(AC_PID_SOURCES
    libraries/AC_PID/AC_HELI_PID.cpp
    libraries/AC_PID/AC_P.cpp
    libraries/AC_PID/AC_PI.cpp
    libraries/AC_PID/AC_PID.cpp
    libraries/AC_PID/AC_PID_2D.cpp
    libraries/AC_PID/AC_PID_Basic.cpp
    libraries/AC_PID/AC_PI_2D.cpp
    libraries/AC_PID/AC_P_1D.cpp
    libraries/AC_PID/AC_P_2D.cpp)

set(AC_PID_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_PID ${AC_PID_SOURCES}) #2
target_include_directories(AC_PID PRIVATE ${AC_PID_INCLUDES})

set_target_properties(AC_PID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_PID mavlink) #clean
target_link_libraries(AC_PID dronecan) #clean
target_link_libraries(AC_PID littlefs) #clean

#------------------------------------------------

set(AC_PrecLand_SOURCES
    libraries/AC_PrecLand/AC_PrecLand_IRLock.cpp
    libraries/AC_PrecLand/AC_PrecLand_MAVLink.cpp
    libraries/AC_PrecLand/AC_PrecLand_SITL.cpp
    libraries/AC_PrecLand/AC_PrecLand_SITL_Gazebo.cpp
    libraries/AC_PrecLand/AC_PrecLand_StateMachine.cpp
    libraries/AC_PrecLand/PosVelEKF.cpp)

set(AC_PrecLand_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_PrecLand ${AC_PrecLand_SOURCES}) #2
target_include_directories(AC_PrecLand PRIVATE ${AC_PrecLand_INCLUDES})

set_target_properties(AC_PrecLand PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_PrecLand mavlink) #clean
target_link_libraries(AC_PrecLand dronecan) #clean
target_link_libraries(AC_PrecLand littlefs) #clean

#------------------------------------------------

set(AC_Sprayer_SOURCES
    libraries/AC_Sprayer/AC_Sprayer.cpp)

set(AC_Sprayer_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_Sprayer ${AC_Sprayer_SOURCES}) #2
target_include_directories(AC_Sprayer PRIVATE ${AC_Sprayer_INCLUDES})

set_target_properties(AC_Sprayer PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_Sprayer mavlink) #clean
target_link_libraries(AC_Sprayer dronecan) #clean
target_link_libraries(AC_Sprayer littlefs) #clean

#------------------------------------------------

set(AC_WPNav_SOURCES
    libraries/AC_WPNav/AC_Circle.cpp
    libraries/AC_WPNav/AC_WPNav.cpp
    libraries/AC_WPNav/AC_WPNav_OA.cpp)

set(AC_WPNav_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_WPNav ${AC_WPNav_SOURCES}) #2
target_include_directories(AC_WPNav PRIVATE ${AC_WPNav_INCLUDES})

set_target_properties(AC_WPNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_WPNav mavlink) #clean
target_link_libraries(AC_WPNav dronecan) #clean
target_link_libraries(AC_WPNav littlefs) #clean

#------------------------------------------------

set(APM_Control_SOURCES
    libraries/APM_Control/AP_AutoTune.cpp
    libraries/APM_Control/AP_FW_Controller.cpp
    libraries/APM_Control/AP_PitchController.cpp
    libraries/APM_Control/AP_RollController.cpp
    libraries/APM_Control/AP_SteerController.cpp
    libraries/APM_Control/AP_YawController.cpp
    libraries/APM_Control/AR_AttitudeControl.cpp
    libraries/APM_Control/AR_PosControl.cpp)

set(APM_Control_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(APM_Control ${APM_Control_SOURCES}) #2
target_include_directories(APM_Control PRIVATE ${APM_Control_INCLUDES})

set_target_properties(APM_Control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(APM_Control mavlink) #clean
target_link_libraries(APM_Control dronecan) #clean
target_link_libraries(APM_Control littlefs) #clean

#------------------------------------------------

set(AP_ADC_SOURCES
    libraries/AP_ADC/AP_ADC_ADS1115.cpp)

set(AP_ADC_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ADC ${AP_ADC_SOURCES}) #2
target_include_directories(AP_ADC PRIVATE ${AP_ADC_INCLUDES})

set_target_properties(AP_ADC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ADC mavlink) #clean
target_link_libraries(AP_ADC dronecan) #clean
target_link_libraries(AP_ADC littlefs) #clean

#------------------------------------------------

set(AP_ADSB_SOURCES
    libraries/AP_ADSB/AP_ADSB_Backend.cpp
    libraries/AP_ADSB/AP_ADSB_Sagetech.cpp
    libraries/AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp
    libraries/AP_ADSB/AP_ADSB_uAvionix_UCP.cpp)

set(AP_ADSB_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ADSB ${AP_ADSB_SOURCES}) #2
target_include_directories(AP_ADSB PRIVATE ${AP_ADSB_INCLUDES})

set_target_properties(AP_ADSB PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ADSB mavlink) #clean
target_link_libraries(AP_ADSB dronecan) #clean
target_link_libraries(AP_ADSB littlefs) #clean

#------------------------------------------------

set(AP_AHRS_SOURCES
    libraries/AP_AHRS/AP_AHRS_External.cpp
    libraries/AP_AHRS/AP_AHRS_Logging.cpp
    libraries/AP_AHRS/AP_AHRS_SIM.cpp
    libraries/AP_AHRS/AP_AHRS_View.cpp)

set(AP_AHRS_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_AHRS ${AP_AHRS_SOURCES}) #2
target_include_directories(AP_AHRS PRIVATE ${AP_AHRS_INCLUDES})

set_target_properties(AP_AHRS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_AHRS mavlink) #clean
target_link_libraries(AP_AHRS dronecan) #clean
target_link_libraries(AP_AHRS littlefs) #clean

#------------------------------------------------

set(AP_AccelCal_SOURCES
    libraries/AP_AccelCal/AP_AccelCal.cpp
    libraries/AP_AccelCal/AccelCalibrator.cpp)

set(AP_AccelCal_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_AccelCal ${AP_AccelCal_SOURCES}) #2
target_include_directories(AP_AccelCal PRIVATE ${AP_AccelCal_INCLUDES})

set_target_properties(AP_AccelCal PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_AccelCal mavlink) #clean
target_link_libraries(AP_AccelCal dronecan) #clean
target_link_libraries(AP_AccelCal littlefs) #clean

#------------------------------------------------

set(AP_AdvancedFailsafe_SOURCES
    libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp)

set(AP_AdvancedFailsafe_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_AdvancedFailsafe ${AP_AdvancedFailsafe_SOURCES}) #2
target_include_directories(AP_AdvancedFailsafe PRIVATE ${AP_AdvancedFailsafe_INCLUDES})

set_target_properties(AP_AdvancedFailsafe PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_AdvancedFailsafe mavlink) #clean
target_link_libraries(AP_AdvancedFailsafe dronecan) #clean
target_link_libraries(AP_AdvancedFailsafe littlefs) #clean

#------------------------------------------------

set(AP_Airspeed_SOURCES
    libraries/AP_Airspeed/AP_Airspeed_ASP5033.cpp
    libraries/AP_Airspeed/AP_Airspeed_AUAV.cpp
    libraries/AP_Airspeed/AP_Airspeed_Backend.cpp
    libraries/AP_Airspeed/AP_Airspeed_DLVR.cpp
    libraries/AP_Airspeed/AP_Airspeed_DroneCAN.cpp
    libraries/AP_Airspeed/AP_Airspeed_External.cpp
    libraries/AP_Airspeed/AP_Airspeed_Health.cpp
    libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp
    libraries/AP_Airspeed/AP_Airspeed_MS5525.cpp
    libraries/AP_Airspeed/AP_Airspeed_MSP.cpp
    libraries/AP_Airspeed/AP_Airspeed_SDP3X.cpp
    libraries/AP_Airspeed/AP_Airspeed_SITL.cpp
    libraries/AP_Airspeed/AP_Airspeed_analog.cpp
    libraries/AP_Airspeed/Airspeed_Calibration.cpp)

set(AP_Airspeed_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Airspeed ${AP_Airspeed_SOURCES}) #2
target_include_directories(AP_Airspeed PRIVATE ${AP_Airspeed_INCLUDES})

set_target_properties(AP_Airspeed PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Airspeed mavlink) #clean
target_link_libraries(AP_Airspeed dronecan) #clean
target_link_libraries(AP_Airspeed littlefs) #clean

#------------------------------------------------

set(AP_Baro_SOURCES
    libraries/AP_Baro/AP_Baro_AUAV.cpp
    libraries/AP_Baro/AP_Baro_BMP085.cpp
    libraries/AP_Baro/AP_Baro_BMP280.cpp
    libraries/AP_Baro/AP_Baro_BMP388.cpp
    libraries/AP_Baro/AP_Baro_BMP581.cpp
    libraries/AP_Baro/AP_Baro_DPS280.cpp
    libraries/AP_Baro/AP_Baro_DroneCAN.cpp
    libraries/AP_Baro/AP_Baro_Dummy.cpp
    libraries/AP_Baro/AP_Baro_ExternalAHRS.cpp
    libraries/AP_Baro/AP_Baro_FBM320.cpp
    libraries/AP_Baro/AP_Baro_HIL.cpp
    libraries/AP_Baro/AP_Baro_ICM20789.cpp
    libraries/AP_Baro/AP_Baro_ICP101XX.cpp
    libraries/AP_Baro/AP_Baro_ICP201XX.cpp
    libraries/AP_Baro/AP_Baro_KellerLD.cpp
    libraries/AP_Baro/AP_Baro_LPS2XH.cpp
    libraries/AP_Baro/AP_Baro_Logging.cpp
    libraries/AP_Baro/AP_Baro_MS5611.cpp
    libraries/AP_Baro/AP_Baro_MSP.cpp
    libraries/AP_Baro/AP_Baro_SPL06.cpp
    libraries/AP_Baro/AP_Baro_Wind.cpp
    libraries/AP_Baro/AP_Baro_atmosphere.cpp)

set(AP_Baro_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Baro ${AP_Baro_SOURCES}) #2
target_include_directories(AP_Baro PRIVATE ${AP_Baro_INCLUDES})

set_target_properties(AP_Baro PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Baro mavlink) #clean
target_link_libraries(AP_Baro dronecan) #clean
target_link_libraries(AP_Baro littlefs) #clean

#------------------------------------------------

set(AP_BattMonitor_SOURCES
    libraries/AP_BattMonitor/AP_BattMonitor_AD7091R5.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Analog.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Backend.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Bebop.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_DroneCAN.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_EFI.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_ESC.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_FuelFlow.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_FuelLevel_Analog.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_FuelLevel_PWM.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Generator.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_INA239.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_INA2xx.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_INA3221.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_LTC2946.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Logging.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_SMBus.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Generic.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_SMBus_NeoDesign.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Rotoye.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_SMBus_SUI.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Solo.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Scripting.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Sum.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Synthetic_Current.cpp
    libraries/AP_BattMonitor/AP_BattMonitor_Torqeedo.cpp)

set(AP_BattMonitor_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_BattMonitor ${AP_BattMonitor_SOURCES}) #2
target_include_directories(AP_BattMonitor PRIVATE ${AP_BattMonitor_INCLUDES})

set_target_properties(AP_BattMonitor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_BattMonitor mavlink) #clean
target_link_libraries(AP_BattMonitor dronecan) #clean
target_link_libraries(AP_BattMonitor littlefs) #clean

#------------------------------------------------

set(AP_Beacon_SOURCES
    libraries/AP_Beacon/AP_Beacon.cpp
    libraries/AP_Beacon/AP_Beacon_Backend.cpp
    libraries/AP_Beacon/AP_Beacon_Marvelmind.cpp
    libraries/AP_Beacon/AP_Beacon_Nooploop.cpp
    libraries/AP_Beacon/AP_Beacon_Pozyx.cpp
    libraries/AP_Beacon/AP_Beacon_SITL.cpp)

set(AP_Beacon_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Beacon ${AP_Beacon_SOURCES}) #2
target_include_directories(AP_Beacon PRIVATE ${AP_Beacon_INCLUDES})

set_target_properties(AP_Beacon PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Beacon mavlink) #clean
target_link_libraries(AP_Beacon dronecan) #clean
target_link_libraries(AP_Beacon littlefs) #clean

#------------------------------------------------

set(AP_BoardConfig_SOURCES
    libraries/AP_BoardConfig/IMU_heater.cpp
    libraries/AP_BoardConfig/board_drivers.cpp)

set(AP_BoardConfig_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_BoardConfig ${AP_BoardConfig_SOURCES}) #2
target_include_directories(AP_BoardConfig PRIVATE ${AP_BoardConfig_INCLUDES})

set_target_properties(AP_BoardConfig PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_BoardConfig mavlink) #clean
target_link_libraries(AP_BoardConfig dronecan) #clean
target_link_libraries(AP_BoardConfig littlefs) #clean

#------------------------------------------------

set(AP_Button_SOURCES
    libraries/AP_Button/AP_Button.cpp)

set(AP_Button_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Button ${AP_Button_SOURCES}) #2
target_include_directories(AP_Button PRIVATE ${AP_Button_INCLUDES})

set_target_properties(AP_Button PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Button mavlink) #clean
target_link_libraries(AP_Button dronecan) #clean
target_link_libraries(AP_Button littlefs) #clean

#------------------------------------------------

set(AP_CANManager_SOURCES
    libraries/AP_CANManager/AP_CANIfaceParams.cpp
    libraries/AP_CANManager/AP_CANManager.cpp
    libraries/AP_CANManager/AP_CANManager_CANDriver_Params.cpp
    libraries/AP_CANManager/AP_CANSensor.cpp
    libraries/AP_CANManager/AP_MAVLinkCAN.cpp
    libraries/AP_CANManager/AP_SLCANIface.cpp)

set(AP_CANManager_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_CANManager ${AP_CANManager_SOURCES}) #2
target_include_directories(AP_CANManager PRIVATE ${AP_CANManager_INCLUDES})

set_target_properties(AP_CANManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_CANManager mavlink) #clean
target_link_libraries(AP_CANManager dronecan) #clean
target_link_libraries(AP_CANManager littlefs) #clean

#------------------------------------------------

set(AP_CSVReader_SOURCES
    libraries/AP_CSVReader/AP_CSVReader.cpp)

set(AP_CSVReader_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_CSVReader ${AP_CSVReader_SOURCES}) #2
target_include_directories(AP_CSVReader PRIVATE ${AP_CSVReader_INCLUDES})

set_target_properties(AP_CSVReader PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_CSVReader mavlink) #clean
target_link_libraries(AP_CSVReader dronecan) #clean
target_link_libraries(AP_CSVReader littlefs) #clean

#------------------------------------------------

set(AP_Camera_SOURCES
    libraries/AP_Camera/AP_Camera.cpp
    libraries/AP_Camera/AP_Camera_Backend.cpp
    libraries/AP_Camera/AP_Camera_Logging.cpp
    libraries/AP_Camera/AP_Camera_MAVLink.cpp
    libraries/AP_Camera/AP_Camera_MAVLinkCamV2.cpp
    libraries/AP_Camera/AP_Camera_Mount.cpp
    libraries/AP_Camera/AP_Camera_Params.cpp
    libraries/AP_Camera/AP_Camera_Relay.cpp
    libraries/AP_Camera/AP_Camera_Scripting.cpp
    libraries/AP_Camera/AP_Camera_Servo.cpp
    libraries/AP_Camera/AP_Camera_SoloGimbal.cpp
    libraries/AP_Camera/AP_RunCam.cpp)

set(AP_Camera_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Camera ${AP_Camera_SOURCES}) #2
target_include_directories(AP_Camera PRIVATE ${AP_Camera_INCLUDES})

set_target_properties(AP_Camera PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Camera mavlink) #clean
target_link_libraries(AP_Camera dronecan) #clean
target_link_libraries(AP_Camera littlefs) #clean

#------------------------------------------------

set(AP_CheckFirmware_SOURCES
    libraries/AP_CheckFirmware/AP_CheckFirmware.cpp
    libraries/AP_CheckFirmware/AP_CheckFirmware_secure_command.cpp
    libraries/AP_CheckFirmware/monocypher.cpp)

set(AP_CheckFirmware_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_CheckFirmware ${AP_CheckFirmware_SOURCES}) #2
target_include_directories(AP_CheckFirmware PRIVATE ${AP_CheckFirmware_INCLUDES})

set_target_properties(AP_CheckFirmware PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_CheckFirmware mavlink) #clean
target_link_libraries(AP_CheckFirmware dronecan) #clean
target_link_libraries(AP_CheckFirmware littlefs) #clean

#------------------------------------------------

set(AP_Common_SOURCES
    libraries/AP_Common/AP_Common.cpp
    libraries/AP_Common/AP_ExpandingArray.cpp
    libraries/AP_Common/AP_FWVersion.cpp
    libraries/AP_Common/ExpandingString.cpp
    libraries/AP_Common/Location.cpp
    libraries/AP_Common/NMEA.cpp
    libraries/AP_Common/c++.cpp
    libraries/AP_Common/float16.cpp
    libraries/AP_Common/sorting.cpp
    libraries/AP_Common/time.cpp)

set(AP_Common_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Common ${AP_Common_SOURCES}) #2
target_include_directories(AP_Common PRIVATE ${AP_Common_INCLUDES})

set_target_properties(AP_Common PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Common mavlink) #clean
target_link_libraries(AP_Common dronecan) #clean
target_link_libraries(AP_Common littlefs) #clean

#------------------------------------------------

set(AP_Compass_SOURCES
    libraries/AP_Compass/AP_Compass.cpp
    libraries/AP_Compass/AP_Compass_AK09916.cpp
    libraries/AP_Compass/AP_Compass_AK8963.cpp
    libraries/AP_Compass/AP_Compass_BMM150.cpp
    libraries/AP_Compass/AP_Compass_BMM350.cpp
    libraries/AP_Compass/AP_Compass_Backend.cpp
    libraries/AP_Compass/AP_Compass_Calibration.cpp
    libraries/AP_Compass/AP_Compass_DroneCAN.cpp
    libraries/AP_Compass/AP_Compass_ExternalAHRS.cpp
    libraries/AP_Compass/AP_Compass_HMC5843.cpp
    libraries/AP_Compass/AP_Compass_IIS2MDC.cpp
    libraries/AP_Compass/AP_Compass_IST8308.cpp
    libraries/AP_Compass/AP_Compass_IST8310.cpp
    libraries/AP_Compass/AP_Compass_LIS2MDL.cpp
    libraries/AP_Compass/AP_Compass_LIS3MDL.cpp
    libraries/AP_Compass/AP_Compass_LSM303D.cpp
    libraries/AP_Compass/AP_Compass_LSM9DS1.cpp
    libraries/AP_Compass/AP_Compass_MAG3110.cpp
    libraries/AP_Compass/AP_Compass_MMC3416.cpp
    libraries/AP_Compass/AP_Compass_MMC5xx3.cpp
    libraries/AP_Compass/AP_Compass_MSP.cpp
    libraries/AP_Compass/AP_Compass_QMC5883L.cpp
    libraries/AP_Compass/AP_Compass_QMC5883P.cpp
    libraries/AP_Compass/AP_Compass_RM3100.cpp
    libraries/AP_Compass/AP_Compass_SITL.cpp
    libraries/AP_Compass/CompassCalibrator.cpp
    libraries/AP_Compass/Compass_PerMotor.cpp
    libraries/AP_Compass/Compass_learn.cpp)

set(AP_Compass_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Compass ${AP_Compass_SOURCES}) #2
target_include_directories(AP_Compass PRIVATE ${AP_Compass_INCLUDES})

set_target_properties(AP_Compass PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Compass mavlink) #clean
target_link_libraries(AP_Compass dronecan) #clean
target_link_libraries(AP_Compass littlefs) #clean

#------------------------------------------------

set(AP_CustomRotations_SOURCES
    libraries/AP_CustomRotations/AP_CustomRotations.cpp
    libraries/AP_CustomRotations/AP_CustomRotations_params.cpp)

set(AP_CustomRotations_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_CustomRotations ${AP_CustomRotations_SOURCES}) #2
target_include_directories(AP_CustomRotations PRIVATE ${AP_CustomRotations_INCLUDES})

set_target_properties(AP_CustomRotations PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_CustomRotations mavlink) #clean
target_link_libraries(AP_CustomRotations dronecan) #clean
target_link_libraries(AP_CustomRotations littlefs) #clean

#------------------------------------------------

set(AP_DAL_SOURCES
    libraries/AP_DAL/AP_DAL_Airspeed.cpp
    libraries/AP_DAL/AP_DAL_Baro.cpp
    libraries/AP_DAL/AP_DAL_Compass.cpp
    libraries/AP_DAL/AP_DAL_GPS.cpp
    libraries/AP_DAL/AP_DAL_InertialSensor.cpp)

set(AP_DAL_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_DAL ${AP_DAL_SOURCES}) #2
target_include_directories(AP_DAL PRIVATE ${AP_DAL_INCLUDES})

set_target_properties(AP_DAL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_DAL mavlink) #clean
target_link_libraries(AP_DAL dronecan) #clean
target_link_libraries(AP_DAL littlefs) #clean

#------------------------------------------------

set(AP_DDS_SOURCES
    libraries/AP_DDS/AP_DDS_Client.cpp
    libraries/AP_DDS/AP_DDS_ExternalControl.cpp
    libraries/AP_DDS/AP_DDS_External_Odom.cpp
    libraries/AP_DDS/AP_DDS_Serial.cpp
    libraries/AP_DDS/AP_DDS_Type_Conversions.cpp
    libraries/AP_DDS/AP_DDS_UDP.cpp)

set(AP_DDS_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_DDS ${AP_DDS_SOURCES}) #2
target_include_directories(AP_DDS PRIVATE ${AP_DDS_INCLUDES})

set_target_properties(AP_DDS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_DDS mavlink) #clean
target_link_libraries(AP_DDS dronecan) #clean
target_link_libraries(AP_DDS littlefs) #clean

#------------------------------------------------

set(AP_Declination_SOURCES
    libraries/AP_Declination/AP_Declination.cpp
    libraries/AP_Declination/tables.cpp)

set(AP_Declination_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Declination ${AP_Declination_SOURCES}) #2
target_include_directories(AP_Declination PRIVATE ${AP_Declination_INCLUDES})

set_target_properties(AP_Declination PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Declination mavlink) #clean
target_link_libraries(AP_Declination dronecan) #clean
target_link_libraries(AP_Declination littlefs) #clean

#------------------------------------------------

set(AP_Devo_Telem_SOURCES
    libraries/AP_Devo_Telem/AP_Devo_Telem.cpp)

set(AP_Devo_Telem_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Devo_Telem ${AP_Devo_Telem_SOURCES}) #2
target_include_directories(AP_Devo_Telem PRIVATE ${AP_Devo_Telem_INCLUDES})

set_target_properties(AP_Devo_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Devo_Telem mavlink) #clean
target_link_libraries(AP_Devo_Telem dronecan) #clean
target_link_libraries(AP_Devo_Telem littlefs) #clean

#------------------------------------------------

set(AP_DroneCAN_SOURCES
    libraries/AP_DroneCAN/AP_Canard_iface.cpp
    libraries/AP_DroneCAN/AP_DroneCAN.cpp
    libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp
    libraries/AP_DroneCAN/AP_DroneCAN_serial.cpp)

set(AP_DroneCAN_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_DroneCAN ${AP_DroneCAN_SOURCES}) #2
target_include_directories(AP_DroneCAN PRIVATE ${AP_DroneCAN_INCLUDES})

set_target_properties(AP_DroneCAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_DroneCAN mavlink) #clean
target_link_libraries(AP_DroneCAN dronecan) #clean
target_link_libraries(AP_DroneCAN littlefs) #clean

#------------------------------------------------

set(AP_EFI_SOURCES
    libraries/AP_EFI/AP_EFI.cpp
    libraries/AP_EFI/AP_EFI_Backend.cpp
    libraries/AP_EFI/AP_EFI_Currawong_ECU.cpp
    libraries/AP_EFI/AP_EFI_DroneCAN.cpp
    libraries/AP_EFI/AP_EFI_Loweheiser.cpp
    libraries/AP_EFI/AP_EFI_MAV.cpp
    libraries/AP_EFI/AP_EFI_NWPMU.cpp
    libraries/AP_EFI/AP_EFI_Scripting.cpp
    libraries/AP_EFI/AP_EFI_Serial_Hirth.cpp
    libraries/AP_EFI/AP_EFI_Serial_Lutan.cpp
    libraries/AP_EFI/AP_EFI_Serial_MS.cpp
    libraries/AP_EFI/AP_EFI_ThrottleLinearisation.cpp)

set(AP_EFI_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_EFI ${AP_EFI_SOURCES}) #2
target_include_directories(AP_EFI PRIVATE ${AP_EFI_INCLUDES})

set_target_properties(AP_EFI PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_EFI mavlink) #clean
target_link_libraries(AP_EFI dronecan) #clean
target_link_libraries(AP_EFI littlefs) #clean

#------------------------------------------------

set(AP_ESC_Telem_SOURCES
    libraries/AP_ESC_Telem/AP_ESC_Telem.cpp
    libraries/AP_ESC_Telem/AP_ESC_Telem_SITL.cpp)

set(AP_ESC_Telem_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ESC_Telem ${AP_ESC_Telem_SOURCES}) #2
target_include_directories(AP_ESC_Telem PRIVATE ${AP_ESC_Telem_INCLUDES})

set_target_properties(AP_ESC_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ESC_Telem mavlink) #clean
target_link_libraries(AP_ESC_Telem dronecan) #clean
target_link_libraries(AP_ESC_Telem littlefs) #clean

#------------------------------------------------

set(AP_ExternalAHRS_SOURCES
    libraries/AP_ExternalAHRS/AP_ExternalAHRS.cpp
    libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain5.cpp
    libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain7.cpp
    libraries/AP_ExternalAHRS/AP_ExternalAHRS_VectorNav.cpp
    libraries/AP_ExternalAHRS/AP_ExternalAHRS_backend.cpp
    libraries/AP_ExternalAHRS/MicroStrain_common.cpp)

set(AP_ExternalAHRS_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ExternalAHRS ${AP_ExternalAHRS_SOURCES}) #2
target_include_directories(AP_ExternalAHRS PRIVATE ${AP_ExternalAHRS_INCLUDES})

set_target_properties(AP_ExternalAHRS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ExternalAHRS mavlink) #clean
target_link_libraries(AP_ExternalAHRS dronecan) #clean
target_link_libraries(AP_ExternalAHRS littlefs) #clean

#------------------------------------------------

set(AP_ExternalControl_SOURCES
    libraries/AP_ExternalControl/AP_ExternalControl.cpp)

set(AP_ExternalControl_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ExternalControl ${AP_ExternalControl_SOURCES}) #2
target_include_directories(AP_ExternalControl PRIVATE ${AP_ExternalControl_INCLUDES})

set_target_properties(AP_ExternalControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ExternalControl mavlink) #clean
target_link_libraries(AP_ExternalControl dronecan) #clean
target_link_libraries(AP_ExternalControl littlefs) #clean

#------------------------------------------------

set(AP_FETtecOneWire_SOURCES
    libraries/AP_FETtecOneWire/AP_FETtecOneWire.cpp)

set(AP_FETtecOneWire_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_FETtecOneWire ${AP_FETtecOneWire_SOURCES}) #2
target_include_directories(AP_FETtecOneWire PRIVATE ${AP_FETtecOneWire_INCLUDES})

set_target_properties(AP_FETtecOneWire PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_FETtecOneWire mavlink) #clean
target_link_libraries(AP_FETtecOneWire dronecan) #clean
target_link_libraries(AP_FETtecOneWire littlefs) #clean

#------------------------------------------------

set(AP_Filesystem_SOURCES
    libraries/AP_Filesystem/AP_Filesystem.cpp
    libraries/AP_Filesystem/AP_Filesystem_ESP32.cpp
    libraries/AP_Filesystem/AP_Filesystem_FATFS.cpp
    libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.cpp
    libraries/AP_Filesystem/AP_Filesystem_Mission.cpp
    libraries/AP_Filesystem/AP_Filesystem_Param.cpp
    libraries/AP_Filesystem/AP_Filesystem_ROMFS.cpp
    libraries/AP_Filesystem/AP_Filesystem_Sys.cpp
    libraries/AP_Filesystem/AP_Filesystem_backend.cpp
    libraries/AP_Filesystem/posix_compat.cpp)

set(AP_Filesystem_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Filesystem ${AP_Filesystem_SOURCES}) #2
target_include_directories(AP_Filesystem PRIVATE ${AP_Filesystem_INCLUDES})

set_target_properties(AP_Filesystem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Filesystem mavlink) #clean
target_link_libraries(AP_Filesystem dronecan) #clean
target_link_libraries(AP_Filesystem littlefs) #clean

#------------------------------------------------

set(AP_FlashStorage_SOURCES
    libraries/AP_FlashStorage/AP_FlashStorage.cpp)

set(AP_FlashStorage_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_FlashStorage ${AP_FlashStorage_SOURCES}) #2
target_include_directories(AP_FlashStorage PRIVATE ${AP_FlashStorage_INCLUDES})

set_target_properties(AP_FlashStorage PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_FlashStorage mavlink) #clean
target_link_libraries(AP_FlashStorage dronecan) #clean
target_link_libraries(AP_FlashStorage littlefs) #clean

#------------------------------------------------

set(AP_Frsky_Telem_SOURCES
    libraries/AP_Frsky_Telem/AP_Frsky_Backend.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_D.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_MAVliteToSPort.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_Message.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_SPortToMAVlite.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_Parameters.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_SPort.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_Telem.cpp)

set(AP_Frsky_Telem_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Frsky_Telem ${AP_Frsky_Telem_SOURCES}) #2
target_include_directories(AP_Frsky_Telem PRIVATE ${AP_Frsky_Telem_INCLUDES})

set_target_properties(AP_Frsky_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Frsky_Telem mavlink) #clean
target_link_libraries(AP_Frsky_Telem dronecan) #clean
target_link_libraries(AP_Frsky_Telem littlefs) #clean

#------------------------------------------------

set(AP_GPS_SOURCES
    libraries/AP_GPS/AP_GPS.cpp
    libraries/AP_GPS/AP_GPS_Blended.cpp
    libraries/AP_GPS/AP_GPS_DroneCAN.cpp
    libraries/AP_GPS/AP_GPS_ERB.cpp
    libraries/AP_GPS/AP_GPS_ExternalAHRS.cpp
    libraries/AP_GPS/AP_GPS_GSOF.cpp
    libraries/AP_GPS/AP_GPS_MAV.cpp
    libraries/AP_GPS/AP_GPS_MSP.cpp
    libraries/AP_GPS/AP_GPS_NMEA.cpp
    libraries/AP_GPS/AP_GPS_NOVA.cpp
    libraries/AP_GPS/AP_GPS_Params.cpp
    libraries/AP_GPS/AP_GPS_SBF.cpp
    libraries/AP_GPS/AP_GPS_SBP.cpp
    libraries/AP_GPS/AP_GPS_SBP2.cpp
    libraries/AP_GPS/AP_GPS_SIRF.cpp
    libraries/AP_GPS/AP_GPS_SITL.cpp
    libraries/AP_GPS/AP_GPS_UBLOX.cpp
    libraries/AP_GPS/GPS_Backend.cpp
    libraries/AP_GPS/MovingBase.cpp
    libraries/AP_GPS/RTCM3_Parser.cpp)

set(AP_GPS_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_GPS ${AP_GPS_SOURCES}) #2
target_include_directories(AP_GPS PRIVATE ${AP_GPS_INCLUDES})

set_target_properties(AP_GPS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_GPS mavlink) #clean
target_link_libraries(AP_GPS dronecan) #clean
target_link_libraries(AP_GPS littlefs) #clean

#------------------------------------------------

set(AP_GSOF_SOURCES
    libraries/AP_GSOF/AP_GSOF.cpp)

set(AP_GSOF_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_GSOF ${AP_GSOF_SOURCES}) #2
target_include_directories(AP_GSOF PRIVATE ${AP_GSOF_INCLUDES})

set_target_properties(AP_GSOF PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_GSOF mavlink) #clean
target_link_libraries(AP_GSOF dronecan) #clean
target_link_libraries(AP_GSOF littlefs) #clean

#------------------------------------------------

set(AP_Generator_SOURCES
    libraries/AP_Generator/AP_Generator.cpp
    libraries/AP_Generator/AP_Generator_Backend.cpp
    libraries/AP_Generator/AP_Generator_Cortex.cpp
    libraries/AP_Generator/AP_Generator_IE_2400.cpp
    libraries/AP_Generator/AP_Generator_IE_650_800.cpp
    libraries/AP_Generator/AP_Generator_IE_FuelCell.cpp
    libraries/AP_Generator/AP_Generator_Loweheiser.cpp
    libraries/AP_Generator/AP_Generator_RichenPower.cpp)

set(AP_Generator_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Generator ${AP_Generator_SOURCES}) #2
target_include_directories(AP_Generator PRIVATE ${AP_Generator_INCLUDES})

set_target_properties(AP_Generator PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Generator mavlink) #clean
target_link_libraries(AP_Generator dronecan) #clean
target_link_libraries(AP_Generator littlefs) #clean

#------------------------------------------------

set(AP_Gripper_SOURCES
    libraries/AP_Gripper/AP_Gripper.cpp
    libraries/AP_Gripper/AP_Gripper_Backend.cpp
    libraries/AP_Gripper/AP_Gripper_EPM.cpp
    libraries/AP_Gripper/AP_Gripper_Servo.cpp)

set(AP_Gripper_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Gripper ${AP_Gripper_SOURCES}) #2
target_include_directories(AP_Gripper PRIVATE ${AP_Gripper_INCLUDES})

set_target_properties(AP_Gripper PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Gripper mavlink) #clean
target_link_libraries(AP_Gripper dronecan) #clean
target_link_libraries(AP_Gripper littlefs) #clean

#------------------------------------------------

set(AP_Hott_Telem_SOURCES
    libraries/AP_Hott_Telem/AP_Hott_Telem.cpp)

set(AP_Hott_Telem_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Hott_Telem ${AP_Hott_Telem_SOURCES}) #2
target_include_directories(AP_Hott_Telem PRIVATE ${AP_Hott_Telem_INCLUDES})

set_target_properties(AP_Hott_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Hott_Telem mavlink) #clean
target_link_libraries(AP_Hott_Telem dronecan) #clean
target_link_libraries(AP_Hott_Telem littlefs) #clean

#------------------------------------------------

set(AP_ICEngine_SOURCES
    libraries/AP_ICEngine/AP_ICEngine.cpp
    libraries/AP_ICEngine/AP_ICEngine_TCA9554.cpp)

set(AP_ICEngine_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ICEngine ${AP_ICEngine_SOURCES}) #2
target_include_directories(AP_ICEngine PRIVATE ${AP_ICEngine_INCLUDES})

set_target_properties(AP_ICEngine PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ICEngine mavlink) #clean
target_link_libraries(AP_ICEngine dronecan) #clean
target_link_libraries(AP_ICEngine littlefs) #clean

#------------------------------------------------

set(AP_IOMCU_SOURCES
    libraries/AP_IOMCU/AP_IOMCU.cpp
    libraries/AP_IOMCU/fw_uploader.cpp)

set(AP_IOMCU_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_IOMCU ${AP_IOMCU_SOURCES}) #2
target_include_directories(AP_IOMCU PRIVATE ${AP_IOMCU_INCLUDES})

set_target_properties(AP_IOMCU PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_IOMCU mavlink) #clean
target_link_libraries(AP_IOMCU dronecan) #clean
target_link_libraries(AP_IOMCU littlefs) #clean

#------------------------------------------------

set(AP_IRLock_SOURCES
    libraries/AP_IRLock/AP_IRLock.cpp
    libraries/AP_IRLock/AP_IRLock_I2C.cpp
    libraries/AP_IRLock/AP_IRLock_SITL.cpp
    libraries/AP_IRLock/AP_IRLock_SITL_Gazebo.cpp)

set(AP_IRLock_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_IRLock ${AP_IRLock_SOURCES}) #2
target_include_directories(AP_IRLock PRIVATE ${AP_IRLock_INCLUDES})

set_target_properties(AP_IRLock PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_IRLock mavlink) #clean
target_link_libraries(AP_IRLock dronecan) #clean
target_link_libraries(AP_IRLock littlefs) #clean

#------------------------------------------------

set(AP_InertialNav_SOURCES
    libraries/AP_InertialNav/AP_InertialNav.cpp)

set(AP_InertialNav_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_InertialNav ${AP_InertialNav_SOURCES}) #2
target_include_directories(AP_InertialNav PRIVATE ${AP_InertialNav_INCLUDES})

set_target_properties(AP_InertialNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_InertialNav mavlink) #clean
target_link_libraries(AP_InertialNav dronecan) #clean
target_link_libraries(AP_InertialNav littlefs) #clean

#------------------------------------------------

set(AP_InertialSensor_SOURCES
    libraries/AP_InertialSensor/AP_InertialSensor_ADIS1647x.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_BMI055.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_BMI088.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_BMI160.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_BMI270.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_ExternalAHRS.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_L3G4200D.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS0.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS1.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_Logging.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_NONE.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_Params.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_SCHA63T.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_SITL.cpp
    libraries/AP_InertialSensor/AP_InertialSensor_tempcal.cpp
    libraries/AP_InertialSensor/AuxiliaryBus.cpp
    libraries/AP_InertialSensor/BatchSampler.cpp)

set(AP_InertialSensor_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_InertialSensor ${AP_InertialSensor_SOURCES}) #2
target_include_directories(AP_InertialSensor PRIVATE ${AP_InertialSensor_INCLUDES})

set_target_properties(AP_InertialSensor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_InertialSensor mavlink) #clean
target_link_libraries(AP_InertialSensor dronecan) #clean
target_link_libraries(AP_InertialSensor littlefs) #clean

#------------------------------------------------

set(AP_InternalError_SOURCES
    libraries/AP_InternalError/AP_InternalError.cpp)

set(AP_InternalError_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_InternalError ${AP_InternalError_SOURCES}) #2
target_include_directories(AP_InternalError PRIVATE ${AP_InternalError_INCLUDES})

set_target_properties(AP_InternalError PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_InternalError mavlink) #clean
target_link_libraries(AP_InternalError dronecan) #clean
target_link_libraries(AP_InternalError littlefs) #clean

#------------------------------------------------

set(AP_JSButton_SOURCES
    libraries/AP_JSButton/AP_JSButton.cpp)

set(AP_JSButton_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_JSButton ${AP_JSButton_SOURCES}) #2
target_include_directories(AP_JSButton PRIVATE ${AP_JSButton_INCLUDES})

set_target_properties(AP_JSButton PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_JSButton mavlink) #clean
target_link_libraries(AP_JSButton dronecan) #clean
target_link_libraries(AP_JSButton littlefs) #clean

#------------------------------------------------

set(AP_JSON_SOURCES
    libraries/AP_JSON/AP_JSON.cpp)

set(AP_JSON_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_JSON ${AP_JSON_SOURCES}) #2
target_include_directories(AP_JSON PRIVATE ${AP_JSON_INCLUDES})

set_target_properties(AP_JSON PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_JSON mavlink) #clean
target_link_libraries(AP_JSON dronecan) #clean
target_link_libraries(AP_JSON littlefs) #clean

#------------------------------------------------

set(AP_KDECAN_SOURCES
    libraries/AP_KDECAN/AP_KDECAN.cpp)

set(AP_KDECAN_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_KDECAN ${AP_KDECAN_SOURCES}) #2
target_include_directories(AP_KDECAN PRIVATE ${AP_KDECAN_INCLUDES})

set_target_properties(AP_KDECAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_KDECAN mavlink) #clean
target_link_libraries(AP_KDECAN dronecan) #clean
target_link_libraries(AP_KDECAN littlefs) #clean

#------------------------------------------------

set(AP_L1_Control_SOURCES
    libraries/AP_L1_Control/AP_L1_Control.cpp)

set(AP_L1_Control_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_L1_Control ${AP_L1_Control_SOURCES}) #2
target_include_directories(AP_L1_Control PRIVATE ${AP_L1_Control_INCLUDES})

set_target_properties(AP_L1_Control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_L1_Control mavlink) #clean
target_link_libraries(AP_L1_Control dronecan) #clean
target_link_libraries(AP_L1_Control littlefs) #clean

#------------------------------------------------

set(AP_LTM_Telem_SOURCES
    libraries/AP_LTM_Telem/AP_LTM_Telem.cpp)

set(AP_LTM_Telem_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_LTM_Telem ${AP_LTM_Telem_SOURCES}) #2
target_include_directories(AP_LTM_Telem PRIVATE ${AP_LTM_Telem_INCLUDES})

set_target_properties(AP_LTM_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_LTM_Telem mavlink) #clean
target_link_libraries(AP_LTM_Telem dronecan) #clean
target_link_libraries(AP_LTM_Telem littlefs) #clean

#------------------------------------------------

set(AP_Landing_SOURCES
    libraries/AP_Landing/AP_Landing.cpp
    libraries/AP_Landing/AP_Landing_Deepstall.cpp)

set(AP_Landing_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Landing ${AP_Landing_SOURCES}) #2
target_include_directories(AP_Landing PRIVATE ${AP_Landing_INCLUDES})

set_target_properties(AP_Landing PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Landing mavlink) #clean
target_link_libraries(AP_Landing dronecan) #clean
target_link_libraries(AP_Landing littlefs) #clean

#------------------------------------------------

set(AP_LeakDetector_SOURCES
    libraries/AP_LeakDetector/AP_LeakDetector.cpp
    libraries/AP_LeakDetector/AP_LeakDetector_Analog.cpp
    libraries/AP_LeakDetector/AP_LeakDetector_Backend.cpp
    libraries/AP_LeakDetector/AP_LeakDetector_Digital.cpp)

set(AP_LeakDetector_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_LeakDetector ${AP_LeakDetector_SOURCES}) #2
target_include_directories(AP_LeakDetector PRIVATE ${AP_LeakDetector_INCLUDES})

set_target_properties(AP_LeakDetector PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_LeakDetector mavlink) #clean
target_link_libraries(AP_LeakDetector dronecan) #clean
target_link_libraries(AP_LeakDetector littlefs) #clean

#------------------------------------------------

set(AP_LightWareSerial_SOURCES
    libraries/AP_LightWareSerial/AP_LightWareSerial.cpp)

set(AP_LightWareSerial_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_LightWareSerial ${AP_LightWareSerial_SOURCES}) #2
target_include_directories(AP_LightWareSerial PRIVATE ${AP_LightWareSerial_INCLUDES})

set_target_properties(AP_LightWareSerial PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_LightWareSerial mavlink) #clean
target_link_libraries(AP_LightWareSerial dronecan) #clean
target_link_libraries(AP_LightWareSerial littlefs) #clean

#------------------------------------------------

set(AP_Logger_SOURCES
    libraries/AP_Logger/AP_Logger_Block.cpp
    libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp
    libraries/AP_Logger/AP_Logger_MAVLink.cpp
    libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp
    libraries/AP_Logger/AP_Logger_W25NXX.cpp
    libraries/AP_Logger/LogFile.cpp)

set(AP_Logger_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Logger ${AP_Logger_SOURCES}) #2
target_include_directories(AP_Logger PRIVATE ${AP_Logger_INCLUDES})

set_target_properties(AP_Logger PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Logger mavlink) #clean
target_link_libraries(AP_Logger dronecan) #clean
target_link_libraries(AP_Logger littlefs) #clean

#------------------------------------------------

set(AP_MSP_SOURCES
    libraries/AP_MSP/AP_MSP.cpp
    libraries/AP_MSP/AP_MSP_Telem_Backend.cpp
    libraries/AP_MSP/AP_MSP_Telem_DJI.cpp
    libraries/AP_MSP/AP_MSP_Telem_DisplayPort.cpp
    libraries/AP_MSP/AP_MSP_Telem_Generic.cpp
    libraries/AP_MSP/msp.cpp
    libraries/AP_MSP/msp_sbuf.cpp)

set(AP_MSP_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_MSP ${AP_MSP_SOURCES}) #2
target_include_directories(AP_MSP PRIVATE ${AP_MSP_INCLUDES})

set_target_properties(AP_MSP PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_MSP mavlink) #clean
target_link_libraries(AP_MSP dronecan) #clean
target_link_libraries(AP_MSP littlefs) #clean

#------------------------------------------------

set(AP_Math_SOURCES
    libraries/AP_Math/AP_GeodesicGrid.cpp
    libraries/AP_Math/AP_Math.cpp
    libraries/AP_Math/SplineCurve.cpp
    libraries/AP_Math/chirp.cpp
    libraries/AP_Math/control.cpp
    libraries/AP_Math/crc.cpp
    libraries/AP_Math/location.cpp
    libraries/AP_Math/location_double.cpp
    libraries/AP_Math/matrix3.cpp
    libraries/AP_Math/matrixN.cpp
    libraries/AP_Math/matrix_alg.cpp
    libraries/AP_Math/polyfit.cpp
    libraries/AP_Math/polygon.cpp
    libraries/AP_Math/quaternion.cpp
    libraries/AP_Math/spline5.cpp
    libraries/AP_Math/vector2.cpp
    libraries/AP_Math/vector3.cpp)

set(AP_Math_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Math ${AP_Math_SOURCES}) #2
target_include_directories(AP_Math PRIVATE ${AP_Math_INCLUDES})

set_target_properties(AP_Math PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Math mavlink) #clean
target_link_libraries(AP_Math dronecan) #clean
target_link_libraries(AP_Math littlefs) #clean

#------------------------------------------------

set(AP_Mission_SOURCES
    libraries/AP_Mission/AP_Mission_ChangeDetector.cpp
    libraries/AP_Mission/AP_Mission_Commands.cpp)

set(AP_Mission_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Mission ${AP_Mission_SOURCES}) #2
target_include_directories(AP_Mission PRIVATE ${AP_Mission_INCLUDES})

set_target_properties(AP_Mission PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Mission mavlink) #clean
target_link_libraries(AP_Mission dronecan) #clean
target_link_libraries(AP_Mission littlefs) #clean

#------------------------------------------------

set(AP_Module_SOURCES
    libraries/AP_Module/AP_Module.cpp)

set(AP_Module_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Module ${AP_Module_SOURCES}) #2
target_include_directories(AP_Module PRIVATE ${AP_Module_INCLUDES})

set_target_properties(AP_Module PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Module mavlink) #clean
target_link_libraries(AP_Module dronecan) #clean
target_link_libraries(AP_Module littlefs) #clean

#------------------------------------------------

set(AP_Motors_SOURCES
    libraries/AP_Motors/AP_Motors6DOF.cpp
    libraries/AP_Motors/AP_MotorsCoax.cpp
    libraries/AP_Motors/AP_MotorsHeli.cpp
    libraries/AP_Motors/AP_MotorsHeli_Dual.cpp
    libraries/AP_Motors/AP_MotorsHeli_Quad.cpp
    libraries/AP_Motors/AP_MotorsHeli_RSC.cpp
    libraries/AP_Motors/AP_MotorsHeli_Single.cpp
    libraries/AP_Motors/AP_MotorsHeli_Swash.cpp
    libraries/AP_Motors/AP_MotorsMatrix_6DoF_Scripting.cpp
    libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.cpp
    libraries/AP_Motors/AP_MotorsSingle.cpp
    libraries/AP_Motors/AP_MotorsTailsitter.cpp
    libraries/AP_Motors/AP_Motors_Class.cpp)

set(AP_Motors_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Motors ${AP_Motors_SOURCES}) #2
target_include_directories(AP_Motors PRIVATE ${AP_Motors_INCLUDES})

set_target_properties(AP_Motors PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Motors mavlink) #clean
target_link_libraries(AP_Motors dronecan) #clean
target_link_libraries(AP_Motors littlefs) #clean

#------------------------------------------------

set(AP_Mount_SOURCES
    libraries/AP_Mount/AP_Mount.cpp
    libraries/AP_Mount/AP_Mount_Alexmos.cpp
    libraries/AP_Mount/AP_Mount_Backend.cpp
    libraries/AP_Mount/AP_Mount_Backend_Serial.cpp
    libraries/AP_Mount/AP_Mount_CADDX.cpp
    libraries/AP_Mount/AP_Mount_MAVLink.cpp
    libraries/AP_Mount/AP_Mount_Params.cpp
    libraries/AP_Mount/AP_Mount_SToRM32.cpp
    libraries/AP_Mount/AP_Mount_SToRM32_serial.cpp
    libraries/AP_Mount/AP_Mount_Scripting.cpp
    libraries/AP_Mount/AP_Mount_Servo.cpp
    libraries/AP_Mount/AP_Mount_Siyi.cpp
    libraries/AP_Mount/AP_Mount_SoloGimbal.cpp
    libraries/AP_Mount/AP_Mount_Topotek.cpp
    libraries/AP_Mount/AP_Mount_Viewpro.cpp
    libraries/AP_Mount/AP_Mount_XFRobot.cpp
    libraries/AP_Mount/AP_Mount_Xacti.cpp
    libraries/AP_Mount/SoloGimbal.cpp
    libraries/AP_Mount/SoloGimbalEKF.cpp
    libraries/AP_Mount/SoloGimbal_Parameters.cpp)

set(AP_Mount_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Mount ${AP_Mount_SOURCES}) #2
target_include_directories(AP_Mount PRIVATE ${AP_Mount_INCLUDES})

set_target_properties(AP_Mount PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Mount mavlink) #clean
target_link_libraries(AP_Mount dronecan) #clean
target_link_libraries(AP_Mount littlefs) #clean

#------------------------------------------------

set(AP_MultiHeap_SOURCES
    libraries/AP_MultiHeap/AP_MultiHeap.cpp
    libraries/AP_MultiHeap/MultiHeap_chibios.cpp
    libraries/AP_MultiHeap/MultiHeap_malloc.cpp)

set(AP_MultiHeap_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_MultiHeap ${AP_MultiHeap_SOURCES}) #2
target_include_directories(AP_MultiHeap PRIVATE ${AP_MultiHeap_INCLUDES})

set_target_properties(AP_MultiHeap PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_MultiHeap mavlink) #clean
target_link_libraries(AP_MultiHeap dronecan) #clean
target_link_libraries(AP_MultiHeap littlefs) #clean

#------------------------------------------------

set(AP_NMEA_Output_SOURCES
    libraries/AP_NMEA_Output/AP_NMEA_Output.cpp)

set(AP_NMEA_Output_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_NMEA_Output ${AP_NMEA_Output_SOURCES}) #2
target_include_directories(AP_NMEA_Output PRIVATE ${AP_NMEA_Output_INCLUDES})

set_target_properties(AP_NMEA_Output PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_NMEA_Output mavlink) #clean
target_link_libraries(AP_NMEA_Output dronecan) #clean
target_link_libraries(AP_NMEA_Output littlefs) #clean

#------------------------------------------------

set(AP_NavEKF_SOURCES
    libraries/AP_NavEKF/AP_NavEKF_Source.cpp
    libraries/AP_NavEKF/AP_NavEKF_core_common.cpp
    libraries/AP_NavEKF/EKFGSF_Logging.cpp
    libraries/AP_NavEKF/EKFGSF_yaw.cpp
    libraries/AP_NavEKF/EKF_Buffer.cpp)

set(AP_NavEKF_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_NavEKF ${AP_NavEKF_SOURCES}) #2
target_include_directories(AP_NavEKF PRIVATE ${AP_NavEKF_INCLUDES})

set_target_properties(AP_NavEKF PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_NavEKF mavlink) #clean
target_link_libraries(AP_NavEKF dronecan) #clean
target_link_libraries(AP_NavEKF littlefs) #clean

#------------------------------------------------

set(AP_NavEKF2_SOURCES
    libraries/AP_NavEKF2/AP_NavEKF2_AirDataFusion.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_Control.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_Logging.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_MagFusion.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_Measurements.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_Outputs.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_PosVelFusion.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_RngBcnFusion.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_VehicleStatus.cpp
    libraries/AP_NavEKF2/AP_NavEKF2_core.cpp
    libraries/AP_NavEKF2/AP_NavEKF_GyroBias.cpp)

set(AP_NavEKF2_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_NavEKF2 ${AP_NavEKF2_SOURCES}) #2
target_include_directories(AP_NavEKF2 PRIVATE ${AP_NavEKF2_INCLUDES})

set_target_properties(AP_NavEKF2 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_NavEKF2 mavlink) #clean
target_link_libraries(AP_NavEKF2 dronecan) #clean
target_link_libraries(AP_NavEKF2 littlefs) #clean

#------------------------------------------------

set(AP_Networking_SOURCES
    libraries/AP_Networking/AP_Networking.cpp
    libraries/AP_Networking/AP_Networking_Backend.cpp
    libraries/AP_Networking/AP_Networking_CAN.cpp
    libraries/AP_Networking/AP_Networking_ChibiOS.cpp
    libraries/AP_Networking/AP_Networking_PPP.cpp
    libraries/AP_Networking/AP_Networking_address.cpp
    libraries/AP_Networking/AP_Networking_macaddr.cpp
    libraries/AP_Networking/AP_Networking_port.cpp
    libraries/AP_Networking/AP_Networking_tests.cpp
    modules/lwip/src/core/altcp.c
    modules/lwip/src/core/altcp_alloc.c
    modules/lwip/src/core/altcp_tcp.c
    modules/lwip/src/core/def.c
    modules/lwip/src/core/dns.c
    modules/lwip/src/core/inet_chksum.c
    modules/lwip/src/core/init.c
    modules/lwip/src/core/ip.c
    modules/lwip/src/core/mem.c
    modules/lwip/src/core/memp.c
    modules/lwip/src/core/netif.c
    modules/lwip/src/core/pbuf.c
    modules/lwip/src/core/raw.c
    modules/lwip/src/core/stats.c
    modules/lwip/src/core/sys.c
    modules/lwip/src/core/tcp.c
    modules/lwip/src/core/tcp_in.c
    modules/lwip/src/core/tcp_out.c
    modules/lwip/src/core/timeouts.c
    modules/lwip/src/core/udp.c
    modules/lwip/src/core/ipv4/acd.c
    modules/lwip/src/core/ipv4/autoip.c
    modules/lwip/src/core/ipv4/dhcp.c
    modules/lwip/src/core/ipv4/etharp.c
    modules/lwip/src/core/ipv4/icmp.c
    modules/lwip/src/core/ipv4/igmp.c
    modules/lwip/src/core/ipv4/ip4.c
    modules/lwip/src/core/ipv4/ip4_addr.c
    modules/lwip/src/core/ipv4/ip4_frag.c
    modules/lwip/src/api/api_lib.c
    modules/lwip/src/api/api_msg.c
    modules/lwip/src/api/err.c
    modules/lwip/src/api/if_api.c
    modules/lwip/src/api/netbuf.c
    modules/lwip/src/api/netdb.c
    modules/lwip/src/api/netifapi.c
    modules/lwip/src/api/sockets.c
    modules/lwip/src/api/tcpip.c
    modules/lwip/src/netif/bridgeif.c
    modules/lwip/src/netif/bridgeif_fdb.c
    modules/lwip/src/netif/ethernet.c
    modules/lwip/src/netif/lowpan6.c
    modules/lwip/src/netif/lowpan6_ble.c
    modules/lwip/src/netif/lowpan6_common.c
    modules/lwip/src/netif/slipif.c
    modules/lwip/src/netif/zepif.c
    modules/lwip/src/netif/ppp/auth.c
    modules/lwip/src/netif/ppp/ccp.c
    modules/lwip/src/netif/ppp/chap-md5.c
    modules/lwip/src/netif/ppp/chap-new.c
    modules/lwip/src/netif/ppp/chap_ms.c
    modules/lwip/src/netif/ppp/demand.c
    modules/lwip/src/netif/ppp/eap.c
    modules/lwip/src/netif/ppp/ecp.c
    modules/lwip/src/netif/ppp/eui64.c
    modules/lwip/src/netif/ppp/fsm.c
    modules/lwip/src/netif/ppp/ipcp.c
    modules/lwip/src/netif/ppp/ipv6cp.c
    modules/lwip/src/netif/ppp/lcp.c
    modules/lwip/src/netif/ppp/magic.c
    modules/lwip/src/netif/ppp/mppe.c
    modules/lwip/src/netif/ppp/multilink.c
    modules/lwip/src/netif/ppp/ppp.c
    modules/lwip/src/netif/ppp/pppapi.c
    modules/lwip/src/netif/ppp/pppcrypt.c
    modules/lwip/src/netif/ppp/pppoe.c
    modules/lwip/src/netif/ppp/pppol2tp.c
    modules/lwip/src/netif/ppp/pppos.c
    modules/lwip/src/netif/ppp/upap.c
    modules/lwip/src/netif/ppp/utils.c
    modules/lwip/src/netif/ppp/vj.c
    libraries/AP_Networking/lwip_hal/arch/sys_arch.cpp)

set(AP_Networking_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Networking ${AP_Networking_SOURCES}) #2
target_include_directories(AP_Networking PRIVATE ${AP_Networking_INCLUDES})

set_target_properties(AP_Networking PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Networking mavlink) #clean
target_link_libraries(AP_Networking dronecan) #clean
target_link_libraries(AP_Networking littlefs) #clean

#------------------------------------------------

set(AP_Notify_SOURCES
    libraries/AP_Notify/AP_BoardLED.cpp
    libraries/AP_Notify/AP_BoardLED2.cpp
    libraries/AP_Notify/AP_Notify.cpp
    libraries/AP_Notify/Buzzer.cpp
    libraries/AP_Notify/DShotLED.cpp
    libraries/AP_Notify/DiscoLED.cpp
    libraries/AP_Notify/DiscreteRGBLed.cpp
    libraries/AP_Notify/Display.cpp
    libraries/AP_Notify/Display_SH1106_I2C.cpp
    libraries/AP_Notify/Display_SITL.cpp
    libraries/AP_Notify/Display_SSD1306_I2C.cpp
    libraries/AP_Notify/DroneCAN_RGB_LED.cpp
    libraries/AP_Notify/ExternalLED.cpp
    libraries/AP_Notify/GPIO_LED_1.cpp
    libraries/AP_Notify/IS31FL3195.cpp
    libraries/AP_Notify/LP5562.cpp
    libraries/AP_Notify/Led_Sysfs.cpp
    libraries/AP_Notify/MMLPlayer.cpp
    libraries/AP_Notify/NCP5623.cpp
    libraries/AP_Notify/NavigatorLED.cpp
    libraries/AP_Notify/NeoPixel.cpp
    libraries/AP_Notify/OreoLED_I2C.cpp
    libraries/AP_Notify/PCA9685LED_I2C.cpp
    libraries/AP_Notify/PixRacerLED.cpp
    libraries/AP_Notify/ProfiLED.cpp
    libraries/AP_Notify/RCOutputRGBLed.cpp
    libraries/AP_Notify/RGBLed.cpp
    libraries/AP_Notify/SITL_SFML_LED.cpp
    libraries/AP_Notify/ScriptingLED.cpp
    libraries/AP_Notify/SerialLED.cpp
    libraries/AP_Notify/ToneAlarm.cpp
    libraries/AP_Notify/ToshibaLED_I2C.cpp
    libraries/AP_Notify/VRBoard_LED.cpp)

set(AP_Notify_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Notify ${AP_Notify_SOURCES}) #2
target_include_directories(AP_Notify PRIVATE ${AP_Notify_INCLUDES})

set_target_properties(AP_Notify PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Notify mavlink) #clean
target_link_libraries(AP_Notify dronecan) #clean
target_link_libraries(AP_Notify littlefs) #clean

#------------------------------------------------

set(AP_OLC_SOURCES
    libraries/AP_OLC/AP_OLC.cpp)

set(AP_OLC_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_OLC ${AP_OLC_SOURCES}) #2
target_include_directories(AP_OLC PRIVATE ${AP_OLC_INCLUDES})

set_target_properties(AP_OLC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_OLC mavlink) #clean
target_link_libraries(AP_OLC dronecan) #clean
target_link_libraries(AP_OLC littlefs) #clean

#------------------------------------------------

set(AP_OSD_SOURCES
    libraries/AP_OSD/AP_OSD.cpp
    libraries/AP_OSD/AP_OSD_Backend.cpp
    libraries/AP_OSD/AP_OSD_MAX7456.cpp
    libraries/AP_OSD/AP_OSD_MSP.cpp
    libraries/AP_OSD/AP_OSD_MSP_DisplayPort.cpp
    libraries/AP_OSD/AP_OSD_SITL.cpp
    libraries/AP_OSD/AP_OSD_Setting.cpp)

set(AP_OSD_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_OSD ${AP_OSD_SOURCES}) #2
target_include_directories(AP_OSD PRIVATE ${AP_OSD_INCLUDES})

set_target_properties(AP_OSD PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_OSD mavlink) #clean
target_link_libraries(AP_OSD dronecan) #clean
target_link_libraries(AP_OSD littlefs) #clean

#------------------------------------------------

set(AP_OpenDroneID_SOURCES
    libraries/AP_OpenDroneID/AP_OpenDroneID.cpp
    libraries/AP_OpenDroneID/AP_OpenDroneID_DroneCAN.cpp)

set(AP_OpenDroneID_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_OpenDroneID ${AP_OpenDroneID_SOURCES}) #2
target_include_directories(AP_OpenDroneID PRIVATE ${AP_OpenDroneID_INCLUDES})

set_target_properties(AP_OpenDroneID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_OpenDroneID mavlink) #clean
target_link_libraries(AP_OpenDroneID dronecan) #clean
target_link_libraries(AP_OpenDroneID littlefs) #clean

#------------------------------------------------

set(AP_OpticalFlow_SOURCES
    libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_Backend.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_CXOF.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_Calibrator.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_HereFlow.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_MSP.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_Onboard.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_PX4Flow.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_Pixart.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_SITL.cpp
    libraries/AP_OpticalFlow/AP_OpticalFlow_UPFLOW.cpp)

set(AP_OpticalFlow_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_OpticalFlow ${AP_OpticalFlow_SOURCES}) #2
target_include_directories(AP_OpticalFlow PRIVATE ${AP_OpticalFlow_INCLUDES})

set_target_properties(AP_OpticalFlow PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_OpticalFlow mavlink) #clean
target_link_libraries(AP_OpticalFlow dronecan) #clean
target_link_libraries(AP_OpticalFlow littlefs) #clean

#------------------------------------------------

set(AP_Parachute_SOURCES
    libraries/AP_Parachute/AP_Parachute.cpp)

set(AP_Parachute_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Parachute ${AP_Parachute_SOURCES}) #2
target_include_directories(AP_Parachute PRIVATE ${AP_Parachute_INCLUDES})

set_target_properties(AP_Parachute PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Parachute mavlink) #clean
target_link_libraries(AP_Parachute dronecan) #clean
target_link_libraries(AP_Parachute littlefs) #clean

#------------------------------------------------

set(AP_Param_SOURCES
    libraries/AP_Param/AP_Param.cpp
    libraries/AP_Param/AP_ParamT.cpp)

set(AP_Param_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Param ${AP_Param_SOURCES}) #2
target_include_directories(AP_Param PRIVATE ${AP_Param_INCLUDES})

set_target_properties(AP_Param PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Param mavlink) #clean
target_link_libraries(AP_Param dronecan) #clean
target_link_libraries(AP_Param littlefs) #clean

#------------------------------------------------

set(AP_PiccoloCAN_SOURCES
    libraries/AP_PiccoloCAN/AP_PiccoloCAN.cpp
    libraries/AP_PiccoloCAN/AP_PiccoloCAN_Cortex.cpp
    libraries/AP_PiccoloCAN/AP_PiccoloCAN_ECU.cpp
    libraries/AP_PiccoloCAN/AP_PiccoloCAN_ESC.cpp
    libraries/AP_PiccoloCAN/AP_PiccoloCAN_Servo.cpp)

set(AP_PiccoloCAN_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_PiccoloCAN ${AP_PiccoloCAN_SOURCES}) #2
target_include_directories(AP_PiccoloCAN PRIVATE ${AP_PiccoloCAN_INCLUDES})

set_target_properties(AP_PiccoloCAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_PiccoloCAN mavlink) #clean
target_link_libraries(AP_PiccoloCAN dronecan) #clean
target_link_libraries(AP_PiccoloCAN littlefs) #clean

#------------------------------------------------

set(AP_Proximity_SOURCES
    libraries/AP_Proximity/AP_Proximity.cpp
    libraries/AP_Proximity/AP_Proximity_AirSimSITL.cpp
    libraries/AP_Proximity/AP_Proximity_Backend.cpp
    libraries/AP_Proximity/AP_Proximity_Backend_Serial.cpp
    libraries/AP_Proximity/AP_Proximity_Boundary_3D.cpp
    libraries/AP_Proximity/AP_Proximity_Cygbot_D1.cpp
    libraries/AP_Proximity/AP_Proximity_DroneCAN.cpp
    libraries/AP_Proximity/AP_Proximity_LD06.cpp
    libraries/AP_Proximity/AP_Proximity_LightWareSF40C.cpp
    libraries/AP_Proximity/AP_Proximity_LightWareSF45B.cpp
    libraries/AP_Proximity/AP_Proximity_MAV.cpp
    libraries/AP_Proximity/AP_Proximity_MR72_CAN.cpp
    libraries/AP_Proximity/AP_Proximity_Params.cpp
    libraries/AP_Proximity/AP_Proximity_RPLidarA2.cpp
    libraries/AP_Proximity/AP_Proximity_RangeFinder.cpp
    libraries/AP_Proximity/AP_Proximity_SITL.cpp
    libraries/AP_Proximity/AP_Proximity_Scripting.cpp
    libraries/AP_Proximity/AP_Proximity_TeraRangerTower.cpp
    libraries/AP_Proximity/AP_Proximity_TeraRangerTowerEvo.cpp)

set(AP_Proximity_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Proximity ${AP_Proximity_SOURCES}) #2
target_include_directories(AP_Proximity PRIVATE ${AP_Proximity_INCLUDES})

set_target_properties(AP_Proximity PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Proximity mavlink) #clean
target_link_libraries(AP_Proximity dronecan) #clean
target_link_libraries(AP_Proximity littlefs) #clean

#------------------------------------------------

set(AP_Quicktune_SOURCES
    libraries/AP_Quicktune/AP_Quicktune.cpp)

set(AP_Quicktune_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Quicktune ${AP_Quicktune_SOURCES}) #2
target_include_directories(AP_Quicktune PRIVATE ${AP_Quicktune_INCLUDES})

set_target_properties(AP_Quicktune PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Quicktune mavlink) #clean
target_link_libraries(AP_Quicktune dronecan) #clean
target_link_libraries(AP_Quicktune littlefs) #clean

#------------------------------------------------

set(AP_RAMTRON_SOURCES
    libraries/AP_RAMTRON/AP_RAMTRON.cpp)

set(AP_RAMTRON_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RAMTRON ${AP_RAMTRON_SOURCES}) #2
target_include_directories(AP_RAMTRON PRIVATE ${AP_RAMTRON_INCLUDES})

set_target_properties(AP_RAMTRON PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RAMTRON mavlink) #clean
target_link_libraries(AP_RAMTRON dronecan) #clean
target_link_libraries(AP_RAMTRON littlefs) #clean

#------------------------------------------------

set(AP_RCMapper_SOURCES
    libraries/AP_RCMapper/AP_RCMapper.cpp)

set(AP_RCMapper_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RCMapper ${AP_RCMapper_SOURCES}) #2
target_include_directories(AP_RCMapper PRIVATE ${AP_RCMapper_INCLUDES})

set_target_properties(AP_RCMapper PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RCMapper mavlink) #clean
target_link_libraries(AP_RCMapper dronecan) #clean
target_link_libraries(AP_RCMapper littlefs) #clean

#------------------------------------------------

set(AP_RCProtocol_SOURCES
    libraries/AP_RCProtocol/AP_RCProtocol_Backend.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_DSM.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_DroneCAN.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_Emlid_RCIO.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_FDM.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_IBUS.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_IOMCU.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_Joystick_SFML.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_MAVLinkRadio.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_PPMSum.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_Radio.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_SBUS.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_SRXL.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_ST24.cpp
    libraries/AP_RCProtocol/AP_RCProtocol_SUMD.cpp
    libraries/AP_RCProtocol/SoftSerial.cpp
    libraries/AP_RCProtocol/spm_srxl.cpp)

set(AP_RCProtocol_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RCProtocol ${AP_RCProtocol_SOURCES}) #2
target_include_directories(AP_RCProtocol PRIVATE ${AP_RCProtocol_INCLUDES})

set_target_properties(AP_RCProtocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RCProtocol mavlink) #clean
target_link_libraries(AP_RCProtocol dronecan) #clean
target_link_libraries(AP_RCProtocol littlefs) #clean

#------------------------------------------------

set(AP_RCTelemetry_SOURCES
    libraries/AP_RCTelemetry/AP_GHST_Telem.cpp
    libraries/AP_RCTelemetry/AP_Spektrum_Telem.cpp)

set(AP_RCTelemetry_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RCTelemetry ${AP_RCTelemetry_SOURCES}) #2
target_include_directories(AP_RCTelemetry PRIVATE ${AP_RCTelemetry_INCLUDES})

set_target_properties(AP_RCTelemetry PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RCTelemetry mavlink) #clean
target_link_libraries(AP_RCTelemetry dronecan) #clean
target_link_libraries(AP_RCTelemetry littlefs) #clean

#------------------------------------------------

set(AP_ROMFS_SOURCES
    libraries/AP_ROMFS/AP_ROMFS.cpp
    libraries/AP_ROMFS/tinflate.cpp)

set(AP_ROMFS_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ROMFS ${AP_ROMFS_SOURCES}) #2
target_include_directories(AP_ROMFS PRIVATE ${AP_ROMFS_INCLUDES})

set_target_properties(AP_ROMFS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ROMFS mavlink) #clean
target_link_libraries(AP_ROMFS dronecan) #clean
target_link_libraries(AP_ROMFS littlefs) #clean

#------------------------------------------------

set(AP_RPM_SOURCES
    libraries/AP_RPM/AP_RPM.cpp
    libraries/AP_RPM/AP_RPM_Params.cpp
    libraries/AP_RPM/RPM_Backend.cpp
    libraries/AP_RPM/RPM_DroneCAN.cpp
    libraries/AP_RPM/RPM_EFI.cpp
    libraries/AP_RPM/RPM_ESC_Telem.cpp
    libraries/AP_RPM/RPM_Generator.cpp
    libraries/AP_RPM/RPM_HarmonicNotch.cpp
    libraries/AP_RPM/RPM_Pin.cpp
    libraries/AP_RPM/RPM_SITL.cpp)

set(AP_RPM_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RPM ${AP_RPM_SOURCES}) #2
target_include_directories(AP_RPM PRIVATE ${AP_RPM_INCLUDES})

set_target_properties(AP_RPM PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RPM mavlink) #clean
target_link_libraries(AP_RPM dronecan) #clean
target_link_libraries(AP_RPM littlefs) #clean

#------------------------------------------------

set(AP_RSSI_SOURCES
    libraries/AP_RSSI/AP_RSSI.cpp)

set(AP_RSSI_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RSSI ${AP_RSSI_SOURCES}) #2
target_include_directories(AP_RSSI PRIVATE ${AP_RSSI_INCLUDES})

set_target_properties(AP_RSSI PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RSSI mavlink) #clean
target_link_libraries(AP_RSSI dronecan) #clean
target_link_libraries(AP_RSSI littlefs) #clean

#------------------------------------------------

set(AP_RTC_SOURCES
    libraries/AP_RTC/AP_RTC.cpp
    libraries/AP_RTC/JitterCorrection.cpp)

set(AP_RTC_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RTC ${AP_RTC_SOURCES}) #2
target_include_directories(AP_RTC PRIVATE ${AP_RTC_INCLUDES})

set_target_properties(AP_RTC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RTC mavlink) #clean
target_link_libraries(AP_RTC dronecan) #clean
target_link_libraries(AP_RTC littlefs) #clean

#------------------------------------------------

set(AP_Radio_SOURCES
    libraries/AP_Radio/AP_Radio.cpp
    libraries/AP_Radio/AP_Radio_backend.cpp
    libraries/AP_Radio/AP_Radio_bk2425.cpp
    libraries/AP_Radio/AP_Radio_cc2500.cpp
    libraries/AP_Radio/AP_Radio_cypress.cpp
    libraries/AP_Radio/driver_bk2425.cpp
    libraries/AP_Radio/driver_cc2500.cpp)

set(AP_Radio_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Radio ${AP_Radio_SOURCES}) #2
target_include_directories(AP_Radio PRIVATE ${AP_Radio_INCLUDES})

set_target_properties(AP_Radio PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Radio mavlink) #clean
target_link_libraries(AP_Radio dronecan) #clean
target_link_libraries(AP_Radio littlefs) #clean

#------------------------------------------------

set(AP_RangeFinder_SOURCES
    libraries/AP_RangeFinder/AP_RangeFinder.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Ainstein_LR_D1.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_BBB_PRU.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_BLPing.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Backend.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Backend_CAN.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Backend_Serial.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Bebop.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Benewake.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Benewake_CAN.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TFMiniPlus.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TFS20L.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_DroneCAN.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_GYUS42v2.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_HC_SR04.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_JRE_Serial.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Lanbao.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_LeddarOne.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_LeddarVu8.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_LightWareI2C.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_LightWare_GRF.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Lua.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_MSP.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarI2CXL.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarSerialLV.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_NMEA.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_NRA24_CAN.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_NoopLoop.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_PWM.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Params.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_PulsedLightLRF.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_RDS02UF.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_SITL.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_TOFSenseF_I2C.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_TOFSenseP_CAN.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_TeraRangerI2C.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_USD1_CAN.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_USD1_Serial.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_VL53L0X.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_VL53L1X.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_Wasp.cpp
    libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp)

set(AP_RangeFinder_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RangeFinder ${AP_RangeFinder_SOURCES}) #2
target_include_directories(AP_RangeFinder PRIVATE ${AP_RangeFinder_INCLUDES})

set_target_properties(AP_RangeFinder PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RangeFinder mavlink) #clean
target_link_libraries(AP_RangeFinder dronecan) #clean
target_link_libraries(AP_RangeFinder littlefs) #clean

#------------------------------------------------

set(AP_Relay_SOURCES
    libraries/AP_Relay/AP_Relay_Params.cpp)

set(AP_Relay_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Relay ${AP_Relay_SOURCES}) #2
target_include_directories(AP_Relay PRIVATE ${AP_Relay_INCLUDES})

set_target_properties(AP_Relay PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Relay mavlink) #clean
target_link_libraries(AP_Relay dronecan) #clean
target_link_libraries(AP_Relay littlefs) #clean

#------------------------------------------------

set(AP_RobotisServo_SOURCES
    libraries/AP_RobotisServo/AP_RobotisServo.cpp)

set(AP_RobotisServo_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_RobotisServo ${AP_RobotisServo_SOURCES}) #2
target_include_directories(AP_RobotisServo PRIVATE ${AP_RobotisServo_INCLUDES})

set_target_properties(AP_RobotisServo PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_RobotisServo mavlink) #clean
target_link_libraries(AP_RobotisServo dronecan) #clean
target_link_libraries(AP_RobotisServo littlefs) #clean

#------------------------------------------------

set(AP_SBusOut_SOURCES
    libraries/AP_SBusOut/AP_SBusOut.cpp)

set(AP_SBusOut_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_SBusOut ${AP_SBusOut_SOURCES}) #2
target_include_directories(AP_SBusOut PRIVATE ${AP_SBusOut_INCLUDES})

set_target_properties(AP_SBusOut PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_SBusOut mavlink) #clean
target_link_libraries(AP_SBusOut dronecan) #clean
target_link_libraries(AP_SBusOut littlefs) #clean

#------------------------------------------------

set(AP_Scheduler_SOURCES
    libraries/AP_Scheduler/PerfInfo.cpp)

set(AP_Scheduler_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Scheduler ${AP_Scheduler_SOURCES}) #2
target_include_directories(AP_Scheduler PRIVATE ${AP_Scheduler_INCLUDES})

set_target_properties(AP_Scheduler PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Scheduler mavlink) #clean
target_link_libraries(AP_Scheduler dronecan) #clean
target_link_libraries(AP_Scheduler littlefs) #clean

#------------------------------------------------

set(AP_SerialLED_SOURCES
    libraries/AP_SerialLED/AP_SerialLED.cpp)

set(AP_SerialLED_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_SerialLED ${AP_SerialLED_SOURCES}) #2
target_include_directories(AP_SerialLED PRIVATE ${AP_SerialLED_INCLUDES})

set_target_properties(AP_SerialLED PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_SerialLED mavlink) #clean
target_link_libraries(AP_SerialLED dronecan) #clean
target_link_libraries(AP_SerialLED littlefs) #clean

#------------------------------------------------

set(AP_SerialManager_SOURCES
    libraries/AP_SerialManager/AP_SerialManager.cpp)

set(AP_SerialManager_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_SerialManager ${AP_SerialManager_SOURCES}) #2
target_include_directories(AP_SerialManager PRIVATE ${AP_SerialManager_INCLUDES})

set_target_properties(AP_SerialManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_SerialManager mavlink) #clean
target_link_libraries(AP_SerialManager dronecan) #clean
target_link_libraries(AP_SerialManager littlefs) #clean

#------------------------------------------------

set(AP_ServoRelayEvents_SOURCES
    libraries/AP_ServoRelayEvents/AP_ServoRelayEvents.cpp)

set(AP_ServoRelayEvents_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ServoRelayEvents ${AP_ServoRelayEvents_SOURCES}) #2
target_include_directories(AP_ServoRelayEvents PRIVATE ${AP_ServoRelayEvents_INCLUDES})

set_target_properties(AP_ServoRelayEvents PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ServoRelayEvents mavlink) #clean
target_link_libraries(AP_ServoRelayEvents dronecan) #clean
target_link_libraries(AP_ServoRelayEvents littlefs) #clean

#------------------------------------------------

set(AP_Servo_Telem_SOURCES
    libraries/AP_Servo_Telem/AP_Servo_Telem.cpp)

set(AP_Servo_Telem_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Servo_Telem ${AP_Servo_Telem_SOURCES}) #2
target_include_directories(AP_Servo_Telem PRIVATE ${AP_Servo_Telem_INCLUDES})

set_target_properties(AP_Servo_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Servo_Telem mavlink) #clean
target_link_libraries(AP_Servo_Telem dronecan) #clean
target_link_libraries(AP_Servo_Telem littlefs) #clean

#------------------------------------------------

set(AP_SmartRTL_SOURCES
    libraries/AP_SmartRTL/AP_SmartRTL.cpp)

set(AP_SmartRTL_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_SmartRTL ${AP_SmartRTL_SOURCES}) #2
target_include_directories(AP_SmartRTL PRIVATE ${AP_SmartRTL_INCLUDES})

set_target_properties(AP_SmartRTL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_SmartRTL mavlink) #clean
target_link_libraries(AP_SmartRTL dronecan) #clean
target_link_libraries(AP_SmartRTL littlefs) #clean

#------------------------------------------------

set(AP_Soaring_SOURCES
    libraries/AP_Soaring/AP_Soaring.cpp
    libraries/AP_Soaring/ExtendedKalmanFilter.cpp
    libraries/AP_Soaring/SpeedToFly.cpp
    libraries/AP_Soaring/Variometer.cpp)

set(AP_Soaring_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Soaring ${AP_Soaring_SOURCES}) #2
target_include_directories(AP_Soaring PRIVATE ${AP_Soaring_INCLUDES})

set_target_properties(AP_Soaring PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Soaring mavlink) #clean
target_link_libraries(AP_Soaring dronecan) #clean
target_link_libraries(AP_Soaring littlefs) #clean

#------------------------------------------------

set(AP_Stats_SOURCES
    libraries/AP_Stats/AP_Stats.cpp)

set(AP_Stats_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Stats ${AP_Stats_SOURCES}) #2
target_include_directories(AP_Stats PRIVATE ${AP_Stats_INCLUDES})

set_target_properties(AP_Stats PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Stats mavlink) #clean
target_link_libraries(AP_Stats dronecan) #clean
target_link_libraries(AP_Stats littlefs) #clean

#------------------------------------------------

set(AP_SurfaceDistance_SOURCES
    libraries/AP_SurfaceDistance/AP_SurfaceDistance.cpp)

set(AP_SurfaceDistance_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_SurfaceDistance ${AP_SurfaceDistance_SOURCES}) #2
target_include_directories(AP_SurfaceDistance PRIVATE ${AP_SurfaceDistance_INCLUDES})

set_target_properties(AP_SurfaceDistance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_SurfaceDistance mavlink) #clean
target_link_libraries(AP_SurfaceDistance dronecan) #clean
target_link_libraries(AP_SurfaceDistance littlefs) #clean

#------------------------------------------------

set(AP_TECS_SOURCES
    libraries/AP_TECS/AP_TECS.cpp)

set(AP_TECS_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_TECS ${AP_TECS_SOURCES}) #2
target_include_directories(AP_TECS PRIVATE ${AP_TECS_INCLUDES})

set_target_properties(AP_TECS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_TECS mavlink) #clean
target_link_libraries(AP_TECS dronecan) #clean
target_link_libraries(AP_TECS littlefs) #clean

#------------------------------------------------

set(AP_TempCalibration_SOURCES
    libraries/AP_TempCalibration/AP_TempCalibration.cpp)

set(AP_TempCalibration_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_TempCalibration ${AP_TempCalibration_SOURCES}) #2
target_include_directories(AP_TempCalibration PRIVATE ${AP_TempCalibration_INCLUDES})

set_target_properties(AP_TempCalibration PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_TempCalibration mavlink) #clean
target_link_libraries(AP_TempCalibration dronecan) #clean
target_link_libraries(AP_TempCalibration littlefs) #clean

#------------------------------------------------

set(AP_TemperatureSensor_SOURCES
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_Analog.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_Backend.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_DroneCAN.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_MAX31865.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_MCP9600.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_MLX90614.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_Params.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_SHT3x.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_TSYS01.cpp
    libraries/AP_TemperatureSensor/AP_TemperatureSensor_TSYS03.cpp)

set(AP_TemperatureSensor_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_TemperatureSensor ${AP_TemperatureSensor_SOURCES}) #2
target_include_directories(AP_TemperatureSensor PRIVATE ${AP_TemperatureSensor_INCLUDES})

set_target_properties(AP_TemperatureSensor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_TemperatureSensor mavlink) #clean
target_link_libraries(AP_TemperatureSensor dronecan) #clean
target_link_libraries(AP_TemperatureSensor littlefs) #clean

#------------------------------------------------

set(AP_Terrain_SOURCES
    libraries/AP_Terrain/TerrainGCS.cpp
    libraries/AP_Terrain/TerrainIO.cpp
    libraries/AP_Terrain/TerrainMission.cpp
    libraries/AP_Terrain/TerrainUtil.cpp)

set(AP_Terrain_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Terrain ${AP_Terrain_SOURCES}) #2
target_include_directories(AP_Terrain PRIVATE ${AP_Terrain_INCLUDES})

set_target_properties(AP_Terrain PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Terrain mavlink) #clean
target_link_libraries(AP_Terrain dronecan) #clean
target_link_libraries(AP_Terrain littlefs) #clean

#------------------------------------------------

set(AP_Torqeedo_SOURCES
    libraries/AP_Torqeedo/AP_Torqeedo.cpp
    libraries/AP_Torqeedo/AP_Torqeedo_Backend.cpp
    libraries/AP_Torqeedo/AP_Torqeedo_Params.cpp
    libraries/AP_Torqeedo/AP_Torqeedo_TQBus.cpp)

set(AP_Torqeedo_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Torqeedo ${AP_Torqeedo_SOURCES}) #2
target_include_directories(AP_Torqeedo PRIVATE ${AP_Torqeedo_INCLUDES})

set_target_properties(AP_Torqeedo PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Torqeedo mavlink) #clean
target_link_libraries(AP_Torqeedo dronecan) #clean
target_link_libraries(AP_Torqeedo littlefs) #clean

#------------------------------------------------

set(AP_Tuning_SOURCES
    libraries/AP_Tuning/AP_Tuning.cpp)

set(AP_Tuning_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Tuning ${AP_Tuning_SOURCES}) #2
target_include_directories(AP_Tuning PRIVATE ${AP_Tuning_INCLUDES})

set_target_properties(AP_Tuning PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Tuning mavlink) #clean
target_link_libraries(AP_Tuning dronecan) #clean
target_link_libraries(AP_Tuning littlefs) #clean

#------------------------------------------------

set(AP_Vehicle_SOURCES
    libraries/AP_Vehicle/Parameters.cpp)

set(AP_Vehicle_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Vehicle ${AP_Vehicle_SOURCES}) #2
target_include_directories(AP_Vehicle PRIVATE ${AP_Vehicle_INCLUDES})

set_target_properties(AP_Vehicle PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Vehicle mavlink) #clean
target_link_libraries(AP_Vehicle dronecan) #clean
target_link_libraries(AP_Vehicle littlefs) #clean

#------------------------------------------------

set(AP_VideoTX_SOURCES
    libraries/AP_VideoTX/AP_SmartAudio.cpp
    libraries/AP_VideoTX/AP_Tramp.cpp
    libraries/AP_VideoTX/AP_VideoTX.cpp)

set(AP_VideoTX_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_VideoTX ${AP_VideoTX_SOURCES}) #2
target_include_directories(AP_VideoTX PRIVATE ${AP_VideoTX_INCLUDES})

set_target_properties(AP_VideoTX PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_VideoTX mavlink) #clean
target_link_libraries(AP_VideoTX dronecan) #clean
target_link_libraries(AP_VideoTX littlefs) #clean

#------------------------------------------------

set(AP_VisualOdom_SOURCES
    libraries/AP_VisualOdom/AP_VisualOdom.cpp
    libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp
    libraries/AP_VisualOdom/AP_VisualOdom_IntelT265.cpp
    libraries/AP_VisualOdom/AP_VisualOdom_Logging.cpp
    libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp)

set(AP_VisualOdom_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_VisualOdom ${AP_VisualOdom_SOURCES}) #2
target_include_directories(AP_VisualOdom PRIVATE ${AP_VisualOdom_INCLUDES})

set_target_properties(AP_VisualOdom PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_VisualOdom mavlink) #clean
target_link_libraries(AP_VisualOdom dronecan) #clean
target_link_libraries(AP_VisualOdom littlefs) #clean

#------------------------------------------------

set(AP_Volz_Protocol_SOURCES
    libraries/AP_Volz_Protocol/AP_Volz_Protocol.cpp)

set(AP_Volz_Protocol_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Volz_Protocol ${AP_Volz_Protocol_SOURCES}) #2
target_include_directories(AP_Volz_Protocol PRIVATE ${AP_Volz_Protocol_INCLUDES})

set_target_properties(AP_Volz_Protocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Volz_Protocol mavlink) #clean
target_link_libraries(AP_Volz_Protocol dronecan) #clean
target_link_libraries(AP_Volz_Protocol littlefs) #clean

#------------------------------------------------

set(AP_WheelEncoder_SOURCES
    libraries/AP_WheelEncoder/AP_WheelEncoder.cpp
    libraries/AP_WheelEncoder/AP_WheelRateControl.cpp
    libraries/AP_WheelEncoder/WheelEncoder_Backend.cpp
    libraries/AP_WheelEncoder/WheelEncoder_Quadrature.cpp
    libraries/AP_WheelEncoder/WheelEncoder_SITL_Quadrature.cpp)

set(AP_WheelEncoder_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_WheelEncoder ${AP_WheelEncoder_SOURCES}) #2
target_include_directories(AP_WheelEncoder PRIVATE ${AP_WheelEncoder_INCLUDES})

set_target_properties(AP_WheelEncoder PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_WheelEncoder mavlink) #clean
target_link_libraries(AP_WheelEncoder dronecan) #clean
target_link_libraries(AP_WheelEncoder littlefs) #clean

#------------------------------------------------

set(AP_Winch_SOURCES
    libraries/AP_Winch/AP_Winch.cpp
    libraries/AP_Winch/AP_Winch_Backend.cpp
    libraries/AP_Winch/AP_Winch_Daiwa.cpp
    libraries/AP_Winch/AP_Winch_PWM.cpp)

set(AP_Winch_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Winch ${AP_Winch_SOURCES}) #2
target_include_directories(AP_Winch PRIVATE ${AP_Winch_INCLUDES})

set_target_properties(AP_Winch PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Winch mavlink) #clean
target_link_libraries(AP_Winch dronecan) #clean
target_link_libraries(AP_Winch littlefs) #clean

#------------------------------------------------

set(AP_WindVane_SOURCES
    libraries/AP_WindVane/AP_WindVane.cpp
    libraries/AP_WindVane/AP_WindVane_Airspeed.cpp
    libraries/AP_WindVane/AP_WindVane_Analog.cpp
    libraries/AP_WindVane/AP_WindVane_Backend.cpp
    libraries/AP_WindVane/AP_WindVane_Home.cpp
    libraries/AP_WindVane/AP_WindVane_ModernDevice.cpp
    libraries/AP_WindVane/AP_WindVane_NMEA.cpp
    libraries/AP_WindVane/AP_WindVane_RPM.cpp
    libraries/AP_WindVane/AP_WindVane_SITL.cpp)

set(AP_WindVane_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_WindVane ${AP_WindVane_SOURCES}) #2
target_include_directories(AP_WindVane PRIVATE ${AP_WindVane_INCLUDES})

set_target_properties(AP_WindVane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_WindVane mavlink) #clean
target_link_libraries(AP_WindVane dronecan) #clean
target_link_libraries(AP_WindVane littlefs) #clean

#------------------------------------------------

set(AR_Motors_SOURCES
    libraries/AR_Motors/AP_MotorsUGV.cpp)

set(AR_Motors_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AR_Motors ${AR_Motors_SOURCES}) #2
target_include_directories(AR_Motors PRIVATE ${AR_Motors_INCLUDES})

set_target_properties(AR_Motors PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AR_Motors mavlink) #clean
target_link_libraries(AR_Motors dronecan) #clean
target_link_libraries(AR_Motors littlefs) #clean

#------------------------------------------------

set(AR_WPNav_SOURCES
    libraries/AR_WPNav/AR_PivotTurn.cpp
    libraries/AR_WPNav/AR_WPNav.cpp
    libraries/AR_WPNav/AR_WPNav_OA.cpp)

set(AR_WPNav_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AR_WPNav ${AR_WPNav_SOURCES}) #2
target_include_directories(AR_WPNav PRIVATE ${AR_WPNav_INCLUDES})

set_target_properties(AR_WPNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AR_WPNav mavlink) #clean
target_link_libraries(AR_WPNav dronecan) #clean
target_link_libraries(AR_WPNav littlefs) #clean

#------------------------------------------------

set(Filter_SOURCES
    libraries/Filter/AP_Filter.cpp
    libraries/Filter/AP_Filter_params.cpp
    libraries/Filter/AP_NotchFilter_params.cpp
    libraries/Filter/DerivativeFilter.cpp
    libraries/Filter/LowPassFilter.cpp
    libraries/Filter/LowPassFilter2p.cpp
    libraries/Filter/ModeFilter.cpp
    libraries/Filter/NotchFilter.cpp
    libraries/Filter/SlewLimiter.cpp)

set(Filter_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(Filter ${Filter_SOURCES}) #2
target_include_directories(Filter PRIVATE ${Filter_INCLUDES})

set_target_properties(Filter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(Filter mavlink) #clean
target_link_libraries(Filter dronecan) #clean
target_link_libraries(Filter littlefs) #clean

#------------------------------------------------

set(GCS_MAVLink_SOURCES
    libraries/GCS_MAVLink/GCS_DeviceOp.cpp
    libraries/GCS_MAVLink/GCS_FTP.cpp
    libraries/GCS_MAVLink/GCS_MAVLink.cpp
    libraries/GCS_MAVLink/GCS_Param.cpp
    libraries/GCS_MAVLink/GCS_Rally.cpp
    libraries/GCS_MAVLink/GCS_ServoRelay.cpp
    libraries/GCS_MAVLink/GCS_Signing.cpp
    libraries/GCS_MAVLink/GCS_serial_control.cpp
    libraries/GCS_MAVLink/MAVLink_routing.cpp
    libraries/GCS_MAVLink/MissionItemProtocol.cpp
    libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp
    libraries/GCS_MAVLink/MissionItemProtocol_Rally.cpp
    libraries/GCS_MAVLink/MissionItemProtocol_Waypoints.cpp)

set(GCS_MAVLink_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(GCS_MAVLink ${GCS_MAVLink_SOURCES}) #2
target_include_directories(GCS_MAVLink PRIVATE ${GCS_MAVLink_INCLUDES})

set_target_properties(GCS_MAVLink PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(GCS_MAVLink mavlink) #clean
target_link_libraries(GCS_MAVLink dronecan) #clean
target_link_libraries(GCS_MAVLink littlefs) #clean

#------------------------------------------------

set(PID_SOURCES
    libraries/PID/PID.cpp)

set(PID_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(PID ${PID_SOURCES}) #2
target_include_directories(PID PRIVATE ${PID_INCLUDES})

set_target_properties(PID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(PID mavlink) #clean
target_link_libraries(PID dronecan) #clean
target_link_libraries(PID littlefs) #clean

#------------------------------------------------

set(RC_Channel_SOURCES
    libraries/RC_Channel/RC_Channels.cpp)

set(RC_Channel_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(RC_Channel ${RC_Channel_SOURCES}) #2
target_include_directories(RC_Channel PRIVATE ${RC_Channel_INCLUDES})

set_target_properties(RC_Channel PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(RC_Channel mavlink) #clean
target_link_libraries(RC_Channel dronecan) #clean
target_link_libraries(RC_Channel littlefs) #clean

#------------------------------------------------

set(SRV_Channel_SOURCES
    libraries/SRV_Channel/SRV_Channel.cpp
    libraries/SRV_Channel/SRV_Channel_aux.cpp
    libraries/SRV_Channel/SRV_Channels.cpp)

set(SRV_Channel_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(SRV_Channel ${SRV_Channel_SOURCES}) #2
target_include_directories(SRV_Channel PRIVATE ${SRV_Channel_INCLUDES})

set_target_properties(SRV_Channel PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(SRV_Channel mavlink) #clean
target_link_libraries(SRV_Channel dronecan) #clean
target_link_libraries(SRV_Channel littlefs) #clean

#------------------------------------------------

set(AP_HAL_SOURCES
    libraries/AP_HAL/CANIface.cpp
    libraries/AP_HAL/DSP.cpp
    libraries/AP_HAL/Device.cpp
    libraries/AP_HAL/GPIO.cpp
    libraries/AP_HAL/HAL.cpp
    libraries/AP_HAL/RCOutput.cpp
    libraries/AP_HAL/Scheduler.cpp
    libraries/AP_HAL/Semaphores.cpp
    libraries/AP_HAL/Storage.cpp
    libraries/AP_HAL/UARTDriver.cpp
    libraries/AP_HAL/Util.cpp
    libraries/AP_HAL/system.cpp
    libraries/AP_HAL/utility/BetterStream.cpp
    libraries/AP_HAL/utility/DataRateLimit.cpp
    libraries/AP_HAL/utility/RCOutput_Tap.cpp
    libraries/AP_HAL/utility/RCOutput_Tap_Linux.cpp
    libraries/AP_HAL/utility/RingBuffer.cpp
    libraries/AP_HAL/utility/Socket.cpp
    libraries/AP_HAL/utility/Socket_native.cpp
    libraries/AP_HAL/utility/dsm.cpp
    libraries/AP_HAL/utility/ftoa_engine.cpp
    libraries/AP_HAL/utility/getopt_cpp.cpp
    libraries/AP_HAL/utility/packetise.cpp
    libraries/AP_HAL/utility/print_vprintf.cpp
    libraries/AP_HAL/utility/replace.cpp
    libraries/AP_HAL/utility/srxl.cpp
    libraries/AP_HAL/utility/st24.cpp
    libraries/AP_HAL/utility/sumd.cpp
    libraries/AP_HAL/utility/utoa_invert.cpp)

set(AP_HAL_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_HAL ${AP_HAL_SOURCES}) #2
target_include_directories(AP_HAL PRIVATE ${AP_HAL_INCLUDES})

set_target_properties(AP_HAL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_HAL mavlink) #clean
target_link_libraries(AP_HAL dronecan) #clean
target_link_libraries(AP_HAL littlefs) #clean

#------------------------------------------------

set(AP_HAL_Empty_SOURCES
    libraries/AP_HAL_Empty/AnalogIn.cpp
    libraries/AP_HAL_Empty/GPIO.cpp
    libraries/AP_HAL_Empty/HAL_Empty_Class.cpp
    libraries/AP_HAL_Empty/RCInput.cpp
    libraries/AP_HAL_Empty/RCOutput.cpp
    libraries/AP_HAL_Empty/Semaphores.cpp
    libraries/AP_HAL_Empty/Storage.cpp
    libraries/AP_HAL_Empty/UARTDriver.cpp)

set(AP_HAL_Empty_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_HAL_Empty ${AP_HAL_Empty_SOURCES}) #2
target_include_directories(AP_HAL_Empty PRIVATE ${AP_HAL_Empty_INCLUDES})

set_target_properties(AP_HAL_Empty PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_HAL_Empty mavlink) #clean
target_link_libraries(AP_HAL_Empty dronecan) #clean
target_link_libraries(AP_HAL_Empty littlefs) #clean

#------------------------------------------------

set(AP_PiccoloCAN_piccolo_protocol_SOURCES
    libraries/AP_PiccoloCAN/piccolo_protocol/CortexDefines.c
    libraries/AP_PiccoloCAN/piccolo_protocol/CortexPackets.c
    libraries/AP_PiccoloCAN/piccolo_protocol/CortexProtocol.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ECUDefines.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ECUPackets.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ECUProtocol.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ECUSettings.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ESCCommands.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ESCDefines.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ESCPackets.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ESCVelocityProtocol.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ServoCommands.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ServoDefines.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ServoPackets.c
    libraries/AP_PiccoloCAN/piccolo_protocol/ServoProtocol.c
    libraries/AP_PiccoloCAN/piccolo_protocol/fielddecode.c
    libraries/AP_PiccoloCAN/piccolo_protocol/fieldencode.c
    libraries/AP_PiccoloCAN/piccolo_protocol/scaleddecode.c
    libraries/AP_PiccoloCAN/piccolo_protocol/scaledencode.c)

set(AP_PiccoloCAN_piccolo_protocol_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_PiccoloCAN_piccolo_protocol ${AP_PiccoloCAN_piccolo_protocol_SOURCES}) #2
target_include_directories(AP_PiccoloCAN_piccolo_protocol PRIVATE ${AP_PiccoloCAN_piccolo_protocol_INCLUDES})

set_target_properties(AP_PiccoloCAN_piccolo_protocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_PiccoloCAN_piccolo_protocol mavlink) #clean
target_link_libraries(AP_PiccoloCAN_piccolo_protocol dronecan) #clean
target_link_libraries(AP_PiccoloCAN_piccolo_protocol littlefs) #clean

#------------------------------------------------

set(AP_HAL_SITL_SOURCES
    libraries/AP_HAL_SITL/AnalogIn.cpp
    libraries/AP_HAL_SITL/CANSocketIface.cpp
    libraries/AP_HAL_SITL/CAN_Multicast.cpp
    libraries/AP_HAL_SITL/CAN_SocketCAN.cpp
    libraries/AP_HAL_SITL/DSP.cpp
    libraries/AP_HAL_SITL/GPIO.cpp
    libraries/AP_HAL_SITL/HAL_SITL_Class.cpp
    libraries/AP_HAL_SITL/I2CDevice.cpp
    libraries/AP_HAL_SITL/RCInput.cpp
    libraries/AP_HAL_SITL/RCOutput.cpp
    libraries/AP_HAL_SITL/SITL_Periph_State.cpp
    libraries/AP_HAL_SITL/SITL_State.cpp
    libraries/AP_HAL_SITL/SITL_State_common.cpp
    libraries/AP_HAL_SITL/SPIDevice.cpp
    libraries/AP_HAL_SITL/Scheduler.cpp
    libraries/AP_HAL_SITL/Semaphores.cpp
    libraries/AP_HAL_SITL/ToneAlarm_SF.cpp
    libraries/AP_HAL_SITL/UART_utils.cpp
    libraries/AP_HAL_SITL/Util.cpp
    libraries/AP_HAL_SITL/sitl_airspeed.cpp
    libraries/AP_HAL_SITL/sitl_rangefinder.cpp
    libraries/AP_HAL_SITL/system.cpp)

set(AP_HAL_SITL_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_HAL_SITL ${AP_HAL_SITL_SOURCES}) #2
target_include_directories(AP_HAL_SITL PRIVATE ${AP_HAL_SITL_INCLUDES})

set_target_properties(AP_HAL_SITL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_HAL_SITL mavlink) #clean
target_link_libraries(AP_HAL_SITL dronecan) #clean
target_link_libraries(AP_HAL_SITL littlefs) #clean

#------------------------------------------------

set(SITL_SOURCES
    libraries/SITL/SIM_ADSB.cpp
    libraries/SITL/SIM_ADSB_Sagetech_MXS.cpp
    libraries/SITL/SIM_AIS.cpp
    libraries/SITL/SIM_AS5600.cpp
    libraries/SITL/SIM_AirSim.cpp
    libraries/SITL/SIM_Airspeed_DLVR.cpp
    libraries/SITL/SIM_BalanceBot.cpp
    libraries/SITL/SIM_Balloon.cpp
    libraries/SITL/SIM_BattMonitor_SMBus.cpp
    libraries/SITL/SIM_BattMonitor_SMBus_Generic.cpp
    libraries/SITL/SIM_BattMonitor_SMBus_Maxell.cpp
    libraries/SITL/SIM_BattMonitor_SMBus_Rotoye.cpp
    libraries/SITL/SIM_Battery.cpp
    libraries/SITL/SIM_Blimp.cpp
    libraries/SITL/SIM_Buzzer.cpp
    libraries/SITL/SIM_CRRCSim.cpp
    libraries/SITL/SIM_CRSF.cpp
    libraries/SITL/SIM_Calibration.cpp
    libraries/SITL/SIM_EFI_Hirth.cpp
    libraries/SITL/SIM_EFI_MegaSquirt.cpp
    libraries/SITL/SIM_ELRS.cpp
    libraries/SITL/SIM_FETtecOneWireESC.cpp
    libraries/SITL/SIM_FlightAxis.cpp
    libraries/SITL/SIM_Frame.cpp
    libraries/SITL/SIM_Frsky.cpp
    libraries/SITL/SIM_Frsky_D.cpp
    libraries/SITL/SIM_GPIO_LED_1.cpp
    libraries/SITL/SIM_GPIO_LED_2.cpp
    libraries/SITL/SIM_GPIO_LED_3.cpp
    libraries/SITL/SIM_GPIO_LED_RGB.cpp
    libraries/SITL/SIM_GPS.cpp
    libraries/SITL/SIM_GPS_FILE.cpp
    libraries/SITL/SIM_GPS_MSP.cpp
    libraries/SITL/SIM_GPS_NMEA.cpp
    libraries/SITL/SIM_GPS_NOVA.cpp
    libraries/SITL/SIM_GPS_SBF.cpp
    libraries/SITL/SIM_GPS_SBP.cpp
    libraries/SITL/SIM_GPS_SBP2.cpp
    libraries/SITL/SIM_GPS_SBP_Common.cpp
    libraries/SITL/SIM_GPS_Trimble.cpp
    libraries/SITL/SIM_GPS_UBLOX.cpp
    libraries/SITL/SIM_Gazebo.cpp
    libraries/SITL/SIM_GeneratorEngine.cpp
    libraries/SITL/SIM_Gimbal.cpp
    libraries/SITL/SIM_Glider.cpp
    libraries/SITL/SIM_Gripper_EPM.cpp
    libraries/SITL/SIM_Gripper_Servo.cpp
    libraries/SITL/SIM_Helicopter.cpp
    libraries/SITL/SIM_I2C.cpp
    libraries/SITL/SIM_I2CDevice.cpp
    libraries/SITL/SIM_ICEngine.cpp
    libraries/SITL/SIM_INA3221.cpp
    libraries/SITL/SIM_IS31FL3195.cpp
    libraries/SITL/SIM_InertialLabs.cpp
    libraries/SITL/SIM_IntelligentEnergy.cpp
    libraries/SITL/SIM_IntelligentEnergy24.cpp
    libraries/SITL/SIM_Invensense_v3.cpp
    libraries/SITL/SIM_JEDEC.cpp
    libraries/SITL/SIM_JEDEC_MX25L3206E.cpp
    libraries/SITL/SIM_JSBSim.cpp
    libraries/SITL/SIM_JSON.cpp
    libraries/SITL/SIM_JSON_Master.cpp
    libraries/SITL/SIM_LED_n.cpp
    libraries/SITL/SIM_LM2755.cpp
    libraries/SITL/SIM_LP5562.cpp
    libraries/SITL/SIM_Loweheiser.cpp
    libraries/SITL/SIM_MS5525.cpp
    libraries/SITL/SIM_MS5611.cpp
    libraries/SITL/SIM_MS5XXX.cpp
    libraries/SITL/SIM_MicroStrain.cpp
    libraries/SITL/SIM_MicroStrain7.cpp
    libraries/SITL/SIM_Morse.cpp
    libraries/SITL/SIM_Motor.cpp
    libraries/SITL/SIM_Multicopter.cpp
    libraries/SITL/SIM_PS_LD06.cpp
    libraries/SITL/SIM_PS_LightWare_SF45B.cpp
    libraries/SITL/SIM_PS_RPLidar.cpp
    libraries/SITL/SIM_PS_TeraRangerTower.cpp
    libraries/SITL/SIM_Parachute.cpp
    libraries/SITL/SIM_Plane.cpp
    libraries/SITL/SIM_Precland.cpp
    libraries/SITL/SIM_QMC5883L.cpp
    libraries/SITL/SIM_QuadPlane.cpp
    libraries/SITL/SIM_RAMTRON.cpp
    libraries/SITL/SIM_RAMTRON_FM25V02.cpp
    libraries/SITL/SIM_RF_Ainstein_LR_D1.cpp
    libraries/SITL/SIM_RF_Ainstein_LR_D1_v19.cpp
    libraries/SITL/SIM_RF_BLping.cpp
    libraries/SITL/SIM_RF_Benewake.cpp
    libraries/SITL/SIM_RF_Benewake_TFMiniPlus.cpp
    libraries/SITL/SIM_RF_GYUS42v2.cpp
    libraries/SITL/SIM_RF_JRE.cpp
    libraries/SITL/SIM_RF_Lanbao.cpp
    libraries/SITL/SIM_RF_LeddarOne.cpp
    libraries/SITL/SIM_RF_LightWareI2C_Legacy16Bit.cpp
    libraries/SITL/SIM_RF_LightWareSerial.cpp
    libraries/SITL/SIM_RF_LightWareSerialBinary.cpp
    libraries/SITL/SIM_RF_LightWare_GRF.cpp
    libraries/SITL/SIM_RF_MAVLink.cpp
    libraries/SITL/SIM_RF_MaxsonarSerialLV.cpp
    libraries/SITL/SIM_RF_NMEA.cpp
    libraries/SITL/SIM_RF_NoopLoop.cpp
    libraries/SITL/SIM_RF_RDS02UF.cpp
    libraries/SITL/SIM_RF_TeraRanger_Serial.cpp
    libraries/SITL/SIM_RF_USD1.cpp
    libraries/SITL/SIM_RF_USD1_v0.cpp
    libraries/SITL/SIM_RF_USD1_v1.cpp
    libraries/SITL/SIM_RF_Wasp.cpp
    libraries/SITL/SIM_RGBLED.cpp
    libraries/SITL/SIM_RichenPower.cpp
    libraries/SITL/SIM_Rover.cpp
    libraries/SITL/SIM_SMBusDevice.cpp
    libraries/SITL/SIM_SPI.cpp
    libraries/SITL/SIM_Sailboat.cpp
    libraries/SITL/SIM_Scrimmage.cpp
    libraries/SITL/SIM_SerialDevice.cpp
    libraries/SITL/SIM_SerialProximitySensor.cpp
    libraries/SITL/SIM_SerialRangeFinder.cpp
    libraries/SITL/SIM_Ship.cpp
    libraries/SITL/SIM_SilentWings.cpp
    libraries/SITL/SIM_SingleCopter.cpp
    libraries/SITL/SIM_SlungPayload.cpp
    libraries/SITL/SIM_SoloGimbal.cpp
    libraries/SITL/SIM_Sprayer.cpp
    libraries/SITL/SIM_StratoBlimp.cpp
    libraries/SITL/SIM_Submarine.cpp
    libraries/SITL/SIM_TFS20L.cpp
    libraries/SITL/SIM_Temperature_MCP9600.cpp
    libraries/SITL/SIM_Temperature_SHT3x.cpp
    libraries/SITL/SIM_Temperature_TSYS01.cpp
    libraries/SITL/SIM_Temperature_TSYS03.cpp
    libraries/SITL/SIM_TeraRangerI2C.cpp
    libraries/SITL/SIM_Tether.cpp
    libraries/SITL/SIM_ToneAlarm.cpp
    libraries/SITL/SIM_ToshibaLED.cpp
    libraries/SITL/SIM_Tracker.cpp
    libraries/SITL/SIM_VectorNav.cpp
    libraries/SITL/SIM_Vicon.cpp
    libraries/SITL/SIM_Volz.cpp
    libraries/SITL/SIM_Webots.cpp
    libraries/SITL/SIM_Webots_Python.cpp
    libraries/SITL/SIM_last_letter.cpp
    libraries/SITL/SITL_Airspeed.cpp
    libraries/SITL/SITL_Baro.cpp
    libraries/SITL/ServoModel.cpp)

set(SITL_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(SITL ${SITL_SOURCES}) #2
target_include_directories(SITL PRIVATE ${SITL_INCLUDES})

set_target_properties(SITL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(SITL mavlink) #clean
target_link_libraries(SITL dronecan) #clean
target_link_libraries(SITL littlefs) #clean

#------------------------------------------------

set(AP_Scripting_SOURCES
    libraries/AP_Scripting/AP_Scripting.cpp
    libraries/AP_Scripting/AP_Scripting_CANSensor.cpp
    libraries/AP_Scripting/AP_Scripting_SerialAccess.cpp
    libraries/AP_Scripting/AP_Scripting_SerialDevice.cpp)

set(AP_Scripting_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Scripting ${AP_Scripting_SOURCES}) #2
target_include_directories(AP_Scripting PRIVATE ${AP_Scripting_INCLUDES})

set_target_properties(AP_Scripting PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Scripting mavlink) #clean
target_link_libraries(AP_Scripting dronecan) #clean
target_link_libraries(AP_Scripting littlefs) #clean

#------------------------------------------------

set(AP_Scripting_lua_src_SOURCES
    libraries/AP_Scripting/lua/src/lapi.c
    libraries/AP_Scripting/lua/src/lauxlib.c
    libraries/AP_Scripting/lua/src/lbaselib.c
    libraries/AP_Scripting/lua/src/lbitlib.c
    libraries/AP_Scripting/lua/src/lcode.c
    libraries/AP_Scripting/lua/src/lcorolib.c
    libraries/AP_Scripting/lua/src/lctype.c
    libraries/AP_Scripting/lua/src/ldblib.c
    libraries/AP_Scripting/lua/src/ldebug.c
    libraries/AP_Scripting/lua/src/ldo.c
    libraries/AP_Scripting/lua/src/ldump.c
    libraries/AP_Scripting/lua/src/lfunc.c
    libraries/AP_Scripting/lua/src/lgc.c
    libraries/AP_Scripting/lua/src/linit.c
    libraries/AP_Scripting/lua/src/liolib.c
    libraries/AP_Scripting/lua/src/llex.c
    libraries/AP_Scripting/lua/src/lmathlib.c
    libraries/AP_Scripting/lua/src/lmem.c
    libraries/AP_Scripting/lua/src/loadlib.c
    libraries/AP_Scripting/lua/src/lobject.c
    libraries/AP_Scripting/lua/src/lopcodes.c
    libraries/AP_Scripting/lua/src/loslib.c
    libraries/AP_Scripting/lua/src/lparser.c
    libraries/AP_Scripting/lua/src/lstate.c
    libraries/AP_Scripting/lua/src/lstring.c
    libraries/AP_Scripting/lua/src/lstrlib.c
    libraries/AP_Scripting/lua/src/ltable.c
    libraries/AP_Scripting/lua/src/ltablib.c
    libraries/AP_Scripting/lua/src/ltm.c
    libraries/AP_Scripting/lua/src/lua.c
    libraries/AP_Scripting/lua/src/luac.c
    libraries/AP_Scripting/lua/src/lundump.c
    libraries/AP_Scripting/lua/src/lutf8lib.c
    libraries/AP_Scripting/lua/src/lvm.c
    libraries/AP_Scripting/lua/src/lzio.c)

set(AP_Scripting_lua_src_INCLUDES
    build/sitl/libraries
    build/sitl/libraries/GCS_MAVLink
    build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
    libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Scripting_lua_src ${AP_Scripting_lua_src_SOURCES}) #2
target_include_directories(AP_Scripting_lua_src PRIVATE ${AP_Scripting_lua_src_INCLUDES})

set_target_properties(AP_Scripting_lua_src PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Scripting_lua_src mavlink) #clean
target_link_libraries(AP_Scripting_lua_src dronecan) #clean
target_link_libraries(AP_Scripting_lua_src littlefs) #clean

#------------------------------------------------

set(AP_NavEKF3_SOURCES
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

set(AP_NavEKF3_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_NavEKF3 ${AP_NavEKF3_SOURCES}) #2
target_compile_definitions(AP_NavEKF3 PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_NavEKF3 PRIVATE ${AP_NavEKF3_INCLUDES})

set_target_properties(AP_NavEKF3 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_NavEKF3 mavlink) #clean
target_link_libraries(AP_NavEKF3 dronecan) #clean
target_link_libraries(AP_NavEKF3 littlefs) #clean

#------------------------------------------------

set(AP_Rally_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(AP_Rally_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Rally ${AP_Rally_SOURCES}) #2
target_compile_definitions(AP_Rally PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_Rally PRIVATE ${AP_Rally_INCLUDES})

set_target_properties(AP_Rally PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Rally mavlink) #clean
target_link_libraries(AP_Rally dronecan) #clean
target_link_libraries(AP_Rally littlefs) #clean

#------------------------------------------------

set(StorageManager_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(StorageManager_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(StorageManager ${StorageManager_SOURCES}) #2
target_compile_definitions(StorageManager PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(StorageManager PRIVATE ${StorageManager_INCLUDES})

set_target_properties(StorageManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(StorageManager mavlink) #clean
target_link_libraries(StorageManager dronecan) #clean
target_link_libraries(StorageManager littlefs) #clean

#------------------------------------------------

set(AP_IBus_Telem_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(AP_IBus_Telem_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_IBus_Telem ${AP_IBus_Telem_SOURCES}) #2
target_compile_definitions(AP_IBus_Telem PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_IBus_Telem PRIVATE ${AP_IBus_Telem_INCLUDES})

set_target_properties(AP_IBus_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_IBus_Telem mavlink) #clean
target_link_libraries(AP_IBus_Telem dronecan) #clean
target_link_libraries(AP_IBus_Telem littlefs) #clean

#------------------------------------------------

set(AP_BLHeli_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(AP_BLHeli_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_BLHeli ${AP_BLHeli_SOURCES}) #2
target_compile_definitions(AP_BLHeli PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_BLHeli PRIVATE ${AP_BLHeli_INCLUDES})

set_target_properties(AP_BLHeli PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_BLHeli mavlink) #clean
target_link_libraries(AP_BLHeli dronecan) #clean
target_link_libraries(AP_BLHeli littlefs) #clean

#------------------------------------------------

set(AC_Fence_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(AC_Fence_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AC_Fence ${AC_Fence_SOURCES}) #2
target_compile_definitions(AC_Fence PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AC_Fence PRIVATE ${AC_Fence_INCLUDES})

set_target_properties(AC_Fence PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AC_Fence mavlink) #clean
target_link_libraries(AC_Fence dronecan) #clean
target_link_libraries(AC_Fence littlefs) #clean

#------------------------------------------------

set(AP_LandingGear_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(AP_LandingGear_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_LandingGear ${AP_LandingGear_SOURCES}) #2
target_compile_definitions(AP_LandingGear PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_LandingGear PRIVATE ${AP_LandingGear_INCLUDES})

set_target_properties(AP_LandingGear PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_LandingGear mavlink) #clean
target_link_libraries(AP_LandingGear dronecan) #clean
target_link_libraries(AP_LandingGear littlefs) #clean

#------------------------------------------------

set(AP_ADSB_sagetech-sdk_SOURCES
    ../libraries/AP_ADSB/sagetech-sdk/appendChecksum.c
    ../libraries/AP_ADSB/sagetech-sdk/calcChecksum.c
    ../libraries/AP_ADSB/sagetech-sdk/charArray2Buf.c
    ../libraries/AP_ADSB/sagetech-sdk/float2Buf.c
    ../libraries/AP_ADSB/sagetech-sdk/icao2Buf.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeAck.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeFlightId.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeInstall.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeMSR.c
    ../libraries/AP_ADSB/sagetech-sdk/sgDecodeSVR.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeDataReq.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeFlightId.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeGPS.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeInstall.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeOperating.c
    ../libraries/AP_ADSB/sagetech-sdk/sgEncodeTargetReq.c
    ../libraries/AP_ADSB/sagetech-sdk/target.c
    ../libraries/AP_ADSB/sagetech-sdk/toAlt.c
    ../libraries/AP_ADSB/sagetech-sdk/toGS.c
    ../libraries/AP_ADSB/sagetech-sdk/toHeading.c
    ../libraries/AP_ADSB/sagetech-sdk/toHeading2.c
    ../libraries/AP_ADSB/sagetech-sdk/toIcao.c
    ../libraries/AP_ADSB/sagetech-sdk/toInt16.c
    ../libraries/AP_ADSB/sagetech-sdk/toInt32.c
    ../libraries/AP_ADSB/sagetech-sdk/toLatLon.c
    ../libraries/AP_ADSB/sagetech-sdk/toTOA.c
    ../libraries/AP_ADSB/sagetech-sdk/toUint16.c
    ../libraries/AP_ADSB/sagetech-sdk/toUint32.c
    ../libraries/AP_ADSB/sagetech-sdk/toVel.c
    ../libraries/AP_ADSB/sagetech-sdk/uint162Buf.c
    ../libraries/AP_ADSB/sagetech-sdk/uint322Buf.c)

set(AP_ADSB_sagetech-sdk_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_ADSB_sagetech-sdk ${AP_ADSB_sagetech-sdk_SOURCES}) #2
target_include_directories(AP_ADSB_sagetech-sdk PRIVATE ${AP_ADSB_sagetech-sdk_INCLUDES})

set_target_properties(AP_ADSB_sagetech-sdk PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_ADSB_sagetech-sdk mavlink) #clean
target_link_libraries(AP_ADSB_sagetech-sdk dronecan) #clean
target_link_libraries(AP_ADSB_sagetech-sdk littlefs) #clean

#------------------------------------------------

set(AP_GyroFFT_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(AP_GyroFFT_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_GyroFFT ${AP_GyroFFT_SOURCES}) #2
target_compile_definitions(AP_GyroFFT PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_GyroFFT PRIVATE ${AP_GyroFFT_INCLUDES})

set_target_properties(AP_GyroFFT PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_GyroFFT mavlink) #clean
target_link_libraries(AP_GyroFFT dronecan) #clean
target_link_libraries(AP_GyroFFT littlefs) #clean

#------------------------------------------------

set(AP_AIS_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(AP_AIS_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_AIS ${AP_AIS_SOURCES}) #2
target_compile_definitions(AP_AIS PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_AIS PRIVATE ${AP_AIS_INCLUDES})

set_target_properties(AP_AIS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_AIS mavlink) #clean
target_link_libraries(AP_AIS dronecan) #clean
target_link_libraries(AP_AIS littlefs) #clean

#------------------------------------------------

set(AP_Arming_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(AP_Arming_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Arming ${AP_Arming_SOURCES}) #2
target_compile_definitions(AP_Arming PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_Arming PRIVATE ${AP_Arming_INCLUDES})

set_target_properties(AP_Arming PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Arming mavlink) #clean
target_link_libraries(AP_Arming dronecan) #clean
target_link_libraries(AP_Arming littlefs) #clean

#------------------------------------------------

set(AP_Follow_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(AP_Follow_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Follow ${AP_Follow_SOURCES}) #2
target_compile_definitions(AP_Follow PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(AP_Follow PRIVATE ${AP_Follow_INCLUDES})

set_target_properties(AP_Follow PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Follow mavlink) #clean
target_link_libraries(AP_Follow dronecan) #clean
target_link_libraries(AP_Follow littlefs) #clean

#------------------------------------------------

set(AntennaTracker_libs_SOURCES
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

set(AntennaTracker_libs_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AntennaTracker_libs ${AntennaTracker_libs_SOURCES}) #2
target_include_directories(AntennaTracker_libs PRIVATE ${AntennaTracker_libs_INCLUDES})

set_target_properties(AntennaTracker_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AntennaTracker_libs AP_AccelCal) #stripped
target_link_libraries(AntennaTracker_libs AP_ADC) #stripped
target_link_libraries(AntennaTracker_libs AP_AHRS) #stripped
target_link_libraries(AntennaTracker_libs AP_Airspeed) #stripped
target_link_libraries(AntennaTracker_libs AP_Baro) #stripped
target_link_libraries(AntennaTracker_libs AP_BattMonitor) #stripped
target_link_libraries(AntennaTracker_libs AP_BoardConfig) #stripped
target_link_libraries(AntennaTracker_libs AP_Camera) #stripped
target_link_libraries(AntennaTracker_libs AP_Common) #stripped
target_link_libraries(AntennaTracker_libs AP_Compass) #stripped
target_link_libraries(AntennaTracker_libs AP_Declination) #stripped
target_link_libraries(AntennaTracker_libs AP_GPS) #stripped
target_link_libraries(AntennaTracker_libs AP_GSOF) #stripped
target_link_libraries(AntennaTracker_libs AP_HAL) #stripped
target_link_libraries(AntennaTracker_libs AP_HAL_Empty) #stripped
target_link_libraries(AntennaTracker_libs AP_DDS) #stripped
target_link_libraries(AntennaTracker_libs AP_InertialSensor) #stripped
target_link_libraries(AntennaTracker_libs AP_Math) #stripped
target_link_libraries(AntennaTracker_libs AP_Mission) #stripped
target_link_libraries(AntennaTracker_libs AP_DAL) #stripped
target_link_libraries(AntennaTracker_libs AP_NavEKF) #stripped
target_link_libraries(AntennaTracker_libs AP_NavEKF2) #stripped
target_link_libraries(AntennaTracker_libs AP_NavEKF3) #stripped
target_link_libraries(AntennaTracker_libs AP_Notify) #stripped
target_link_libraries(AntennaTracker_libs AP_OpticalFlow) #stripped
target_link_libraries(AntennaTracker_libs AP_Param) #stripped
target_link_libraries(AntennaTracker_libs AP_Rally) #stripped
target_link_libraries(AntennaTracker_libs AP_LightWareSerial) #stripped
target_link_libraries(AntennaTracker_libs AP_RangeFinder) #stripped
target_link_libraries(AntennaTracker_libs AP_Scheduler) #stripped
target_link_libraries(AntennaTracker_libs AP_SerialManager) #stripped
target_link_libraries(AntennaTracker_libs AP_Terrain) #stripped
target_link_libraries(AntennaTracker_libs AP_Vehicle) #stripped
target_link_libraries(AntennaTracker_libs AP_InternalError) #stripped
target_link_libraries(AntennaTracker_libs AP_Logger) #stripped
target_link_libraries(AntennaTracker_libs Filter) #stripped
target_link_libraries(AntennaTracker_libs GCS_MAVLink) #stripped
target_link_libraries(AntennaTracker_libs RC_Channel) #stripped
target_link_libraries(AntennaTracker_libs SRV_Channel) #stripped
target_link_libraries(AntennaTracker_libs StorageManager) #stripped
target_link_libraries(AntennaTracker_libs AP_Tuning) #stripped
target_link_libraries(AntennaTracker_libs AP_RPM) #stripped
target_link_libraries(AntennaTracker_libs AP_RSSI) #stripped
target_link_libraries(AntennaTracker_libs AP_Mount) #stripped
target_link_libraries(AntennaTracker_libs AP_Module) #stripped
target_link_libraries(AntennaTracker_libs AP_Button) #stripped
target_link_libraries(AntennaTracker_libs AP_ICEngine) #stripped
target_link_libraries(AntennaTracker_libs AP_Networking) #stripped
target_link_libraries(AntennaTracker_libs AP_Frsky_Telem) #stripped
target_link_libraries(AntennaTracker_libs AP_IBus_Telem) #stripped
target_link_libraries(AntennaTracker_libs AP_FlashStorage) #stripped
target_link_libraries(AntennaTracker_libs AP_Relay) #stripped
target_link_libraries(AntennaTracker_libs AP_ServoRelayEvents) #stripped
target_link_libraries(AntennaTracker_libs AP_Volz_Protocol) #stripped
target_link_libraries(AntennaTracker_libs AP_SBusOut) #stripped
target_link_libraries(AntennaTracker_libs AP_IOMCU) #stripped
target_link_libraries(AntennaTracker_libs AP_Parachute) #stripped
target_link_libraries(AntennaTracker_libs AP_RAMTRON) #stripped
target_link_libraries(AntennaTracker_libs AP_RCProtocol) #stripped
target_link_libraries(AntennaTracker_libs AP_Radio) #stripped
target_link_libraries(AntennaTracker_libs AP_TempCalibration) #stripped
target_link_libraries(AntennaTracker_libs AP_VisualOdom) #stripped
target_link_libraries(AntennaTracker_libs AP_BLHeli) #stripped
target_link_libraries(AntennaTracker_libs AP_ROMFS) #stripped
target_link_libraries(AntennaTracker_libs AP_Proximity) #stripped
target_link_libraries(AntennaTracker_libs AP_Gripper) #stripped
target_link_libraries(AntennaTracker_libs AP_RTC) #stripped
target_link_libraries(AntennaTracker_libs AC_Sprayer) #stripped
target_link_libraries(AntennaTracker_libs AC_Fence) #stripped
target_link_libraries(AntennaTracker_libs AC_Avoidance) #stripped
target_link_libraries(AntennaTracker_libs AP_LandingGear) #stripped
target_link_libraries(AntennaTracker_libs AP_RobotisServo) #stripped
target_link_libraries(AntennaTracker_libs AP_NMEA_Output) #stripped
target_link_libraries(AntennaTracker_libs AP_OSD) #stripped
target_link_libraries(AntennaTracker_libs AP_Filesystem) #stripped
target_link_libraries(AntennaTracker_libs AP_ADSB) #stripped
target_link_libraries(AntennaTracker_libs AC_PID) #stripped
target_link_libraries(AntennaTracker_libs AP_SerialLED) #stripped
target_link_libraries(AntennaTracker_libs AP_EFI) #stripped
target_link_libraries(AntennaTracker_libs AP_Hott_Telem) #stripped
target_link_libraries(AntennaTracker_libs AP_ESC_Telem) #stripped
target_link_libraries(AntennaTracker_libs AP_Servo_Telem) #stripped
target_link_libraries(AntennaTracker_libs AP_Stats) #stripped
target_link_libraries(AntennaTracker_libs AP_GyroFFT) #stripped
target_link_libraries(AntennaTracker_libs AP_RCTelemetry) #stripped
target_link_libraries(AntennaTracker_libs AP_Generator) #stripped
target_link_libraries(AntennaTracker_libs AP_MSP) #stripped
target_link_libraries(AntennaTracker_libs AP_OLC) #stripped
target_link_libraries(AntennaTracker_libs AP_WheelEncoder) #stripped
target_link_libraries(AntennaTracker_libs AP_ExternalAHRS) #stripped
target_link_libraries(AntennaTracker_libs AP_VideoTX) #stripped
target_link_libraries(AntennaTracker_libs AP_FETtecOneWire) #stripped
target_link_libraries(AntennaTracker_libs AP_TemperatureSensor) #stripped
target_link_libraries(AntennaTracker_libs AP_Torqeedo) #stripped
target_link_libraries(AntennaTracker_libs AP_CustomRotations) #stripped
target_link_libraries(AntennaTracker_libs AP_AIS) #stripped
target_link_libraries(AntennaTracker_libs AP_OpenDroneID) #stripped
target_link_libraries(AntennaTracker_libs AP_CheckFirmware) #stripped
target_link_libraries(AntennaTracker_libs AP_ExternalControl) #stripped
target_link_libraries(AntennaTracker_libs AP_JSON) #stripped
target_link_libraries(AntennaTracker_libs AP_Beacon) #stripped
target_link_libraries(AntennaTracker_libs AP_Arming) #stripped
target_link_libraries(AntennaTracker_libs AP_RCMapper) #stripped
target_link_libraries(AntennaTracker_libs AP_MultiHeap) #stripped
target_link_libraries(AntennaTracker_libs AP_Follow) #stripped
target_link_libraries(AntennaTracker_libs AP_CANManager) #stripped
target_link_libraries(AntennaTracker_libs AP_KDECAN) #stripped
target_link_libraries(AntennaTracker_libs AP_PiccoloCAN) #stripped
target_link_libraries(AntennaTracker_libs AP_DroneCAN) #stripped
target_link_libraries(AntennaTracker_libs AP_HAL_SITL) #stripped
target_link_libraries(AntennaTracker_libs AP_CSVReader) #stripped
target_link_libraries(AntennaTracker_libs SITL) #stripped
target_link_libraries(AntennaTracker_libs AP_Scripting) #stripped
target_link_libraries(AntennaTracker_libs AC_CustomControl) #stripped

#------------------------------------------------

set(bin_antennatracker_SOURCES
    AntennaTracker/GCS_MAVLink_Tracker.cpp
    AntennaTracker/GCS_Tracker.cpp
    AntennaTracker/Log.cpp
    AntennaTracker/Parameters.cpp
    AntennaTracker/RC_Channel_Tracker.cpp
    AntennaTracker/Tracker.cpp
    AntennaTracker/mode.cpp
    AntennaTracker/mode_auto.cpp
    AntennaTracker/mode_guided.cpp
    AntennaTracker/mode_manual.cpp
    AntennaTracker/mode_scan.cpp
    AntennaTracker/mode_servotest.cpp
    AntennaTracker/radio.cpp
    AntennaTracker/sensors.cpp
    AntennaTracker/servos.cpp
    AntennaTracker/system.cpp
    AntennaTracker/tracking.cpp)

set(bin_antennatracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(bin_antennatracker ${bin_antennatracker_SOURCES})
#target_compile_definitions(bin_antennatracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
#target_include_directories(bin_antennatracker PRIVATE ${bin_antennatracker_INCLUDES})


target_link_libraries(bin_antennatracker AntennaTracker_libs) #clean

#------------------------------------------------

set(AP_Avoidance_SOURCES
    ../libraries/AP_Avoidance/AP_Avoidance.cpp)

set(AP_Avoidance_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(AP_Avoidance ${AP_Avoidance_SOURCES}) #2
target_compile_definitions(AP_Avoidance PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(AP_Avoidance PRIVATE ${AP_Avoidance_INCLUDES})

set_target_properties(AP_Avoidance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(AP_Avoidance mavlink) #clean
target_link_libraries(AP_Avoidance dronecan) #clean
target_link_libraries(AP_Avoidance littlefs) #clean

#------------------------------------------------

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

add_library(ArduCopter_libs ${ArduCopter_libs_SOURCES}) #2
target_include_directories(ArduCopter_libs PRIVATE ${ArduCopter_libs_INCLUDES})

set_target_properties(ArduCopter_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(ArduCopter_libs AP_AccelCal) #stripped
target_link_libraries(ArduCopter_libs AP_ADC) #stripped
target_link_libraries(ArduCopter_libs AP_AHRS) #stripped
target_link_libraries(ArduCopter_libs AP_Airspeed) #stripped
target_link_libraries(ArduCopter_libs AP_Baro) #stripped
target_link_libraries(ArduCopter_libs AP_BattMonitor) #stripped
target_link_libraries(ArduCopter_libs AP_BoardConfig) #stripped
target_link_libraries(ArduCopter_libs AP_Camera) #stripped
target_link_libraries(ArduCopter_libs AP_Common) #stripped
target_link_libraries(ArduCopter_libs AP_Compass) #stripped
target_link_libraries(ArduCopter_libs AP_Declination) #stripped
target_link_libraries(ArduCopter_libs AP_GPS) #stripped
target_link_libraries(ArduCopter_libs AP_GSOF) #stripped
target_link_libraries(ArduCopter_libs AP_HAL) #stripped
target_link_libraries(ArduCopter_libs AP_HAL_Empty) #stripped
target_link_libraries(ArduCopter_libs AP_DDS) #stripped
target_link_libraries(ArduCopter_libs AP_InertialSensor) #stripped
target_link_libraries(ArduCopter_libs AP_Math) #stripped
target_link_libraries(ArduCopter_libs AP_Mission) #stripped
target_link_libraries(ArduCopter_libs AP_DAL) #stripped
target_link_libraries(ArduCopter_libs AP_NavEKF) #stripped
target_link_libraries(ArduCopter_libs AP_NavEKF2) #stripped
target_link_libraries(ArduCopter_libs AP_NavEKF3) #stripped
target_link_libraries(ArduCopter_libs AP_Notify) #stripped
target_link_libraries(ArduCopter_libs AP_OpticalFlow) #stripped
target_link_libraries(ArduCopter_libs AP_Param) #stripped
target_link_libraries(ArduCopter_libs AP_Rally) #stripped
target_link_libraries(ArduCopter_libs AP_LightWareSerial) #stripped
target_link_libraries(ArduCopter_libs AP_RangeFinder) #stripped
target_link_libraries(ArduCopter_libs AP_Scheduler) #stripped
target_link_libraries(ArduCopter_libs AP_SerialManager) #stripped
target_link_libraries(ArduCopter_libs AP_Terrain) #stripped
target_link_libraries(ArduCopter_libs AP_Vehicle) #stripped
target_link_libraries(ArduCopter_libs AP_InternalError) #stripped
target_link_libraries(ArduCopter_libs AP_Logger) #stripped
target_link_libraries(ArduCopter_libs Filter) #stripped
target_link_libraries(ArduCopter_libs GCS_MAVLink) #stripped
target_link_libraries(ArduCopter_libs RC_Channel) #stripped
target_link_libraries(ArduCopter_libs SRV_Channel) #stripped
target_link_libraries(ArduCopter_libs StorageManager) #stripped
target_link_libraries(ArduCopter_libs AP_Tuning) #stripped
target_link_libraries(ArduCopter_libs AP_RPM) #stripped
target_link_libraries(ArduCopter_libs AP_RSSI) #stripped
target_link_libraries(ArduCopter_libs AP_Mount) #stripped
target_link_libraries(ArduCopter_libs AP_Module) #stripped
target_link_libraries(ArduCopter_libs AP_Button) #stripped
target_link_libraries(ArduCopter_libs AP_ICEngine) #stripped
target_link_libraries(ArduCopter_libs AP_Networking) #stripped
target_link_libraries(ArduCopter_libs AP_Frsky_Telem) #stripped
target_link_libraries(ArduCopter_libs AP_IBus_Telem) #stripped
target_link_libraries(ArduCopter_libs AP_FlashStorage) #stripped
target_link_libraries(ArduCopter_libs AP_Relay) #stripped
target_link_libraries(ArduCopter_libs AP_ServoRelayEvents) #stripped
target_link_libraries(ArduCopter_libs AP_Volz_Protocol) #stripped
target_link_libraries(ArduCopter_libs AP_SBusOut) #stripped
target_link_libraries(ArduCopter_libs AP_IOMCU) #stripped
target_link_libraries(ArduCopter_libs AP_Parachute) #stripped
target_link_libraries(ArduCopter_libs AP_RAMTRON) #stripped
target_link_libraries(ArduCopter_libs AP_RCProtocol) #stripped
target_link_libraries(ArduCopter_libs AP_Radio) #stripped
target_link_libraries(ArduCopter_libs AP_TempCalibration) #stripped
target_link_libraries(ArduCopter_libs AP_VisualOdom) #stripped
target_link_libraries(ArduCopter_libs AP_BLHeli) #stripped
target_link_libraries(ArduCopter_libs AP_ROMFS) #stripped
target_link_libraries(ArduCopter_libs AP_Proximity) #stripped
target_link_libraries(ArduCopter_libs AP_Gripper) #stripped
target_link_libraries(ArduCopter_libs AP_RTC) #stripped
target_link_libraries(ArduCopter_libs AC_Sprayer) #stripped
target_link_libraries(ArduCopter_libs AC_Fence) #stripped
target_link_libraries(ArduCopter_libs AC_Avoidance) #stripped
target_link_libraries(ArduCopter_libs AP_LandingGear) #stripped
target_link_libraries(ArduCopter_libs AP_RobotisServo) #stripped
target_link_libraries(ArduCopter_libs AP_NMEA_Output) #stripped
target_link_libraries(ArduCopter_libs AP_OSD) #stripped
target_link_libraries(ArduCopter_libs AP_Filesystem) #stripped
target_link_libraries(ArduCopter_libs AP_ADSB) #stripped
target_link_libraries(ArduCopter_libs AC_PID) #stripped
target_link_libraries(ArduCopter_libs AP_SerialLED) #stripped
target_link_libraries(ArduCopter_libs AP_EFI) #stripped
target_link_libraries(ArduCopter_libs AP_Hott_Telem) #stripped
target_link_libraries(ArduCopter_libs AP_ESC_Telem) #stripped
target_link_libraries(ArduCopter_libs AP_Servo_Telem) #stripped
target_link_libraries(ArduCopter_libs AP_Stats) #stripped
target_link_libraries(ArduCopter_libs AP_GyroFFT) #stripped
target_link_libraries(ArduCopter_libs AP_RCTelemetry) #stripped
target_link_libraries(ArduCopter_libs AP_Generator) #stripped
target_link_libraries(ArduCopter_libs AP_MSP) #stripped
target_link_libraries(ArduCopter_libs AP_OLC) #stripped
target_link_libraries(ArduCopter_libs AP_WheelEncoder) #stripped
target_link_libraries(ArduCopter_libs AP_ExternalAHRS) #stripped
target_link_libraries(ArduCopter_libs AP_VideoTX) #stripped
target_link_libraries(ArduCopter_libs AP_FETtecOneWire) #stripped
target_link_libraries(ArduCopter_libs AP_TemperatureSensor) #stripped
target_link_libraries(ArduCopter_libs AP_Torqeedo) #stripped
target_link_libraries(ArduCopter_libs AP_CustomRotations) #stripped
target_link_libraries(ArduCopter_libs AP_AIS) #stripped
target_link_libraries(ArduCopter_libs AP_OpenDroneID) #stripped
target_link_libraries(ArduCopter_libs AP_CheckFirmware) #stripped
target_link_libraries(ArduCopter_libs AP_ExternalControl) #stripped
target_link_libraries(ArduCopter_libs AP_JSON) #stripped
target_link_libraries(ArduCopter_libs AP_Beacon) #stripped
target_link_libraries(ArduCopter_libs AP_Arming) #stripped
target_link_libraries(ArduCopter_libs AP_RCMapper) #stripped
target_link_libraries(ArduCopter_libs AP_MultiHeap) #stripped
target_link_libraries(ArduCopter_libs AP_Follow) #stripped
target_link_libraries(ArduCopter_libs AP_CANManager) #stripped
target_link_libraries(ArduCopter_libs AP_KDECAN) #stripped
target_link_libraries(ArduCopter_libs AP_PiccoloCAN) #stripped
target_link_libraries(ArduCopter_libs AC_AttitudeControl) #stripped
target_link_libraries(ArduCopter_libs AC_InputManager) #stripped
target_link_libraries(ArduCopter_libs AC_PrecLand) #stripped
target_link_libraries(ArduCopter_libs AC_Autorotation) #stripped
target_link_libraries(ArduCopter_libs AC_WPNav) #stripped
target_link_libraries(ArduCopter_libs AP_IRLock) #stripped
target_link_libraries(ArduCopter_libs AP_Motors) #stripped
target_link_libraries(ArduCopter_libs AP_Avoidance) #stripped
target_link_libraries(ArduCopter_libs AP_AdvancedFailsafe) #stripped
target_link_libraries(ArduCopter_libs AP_SmartRTL) #stripped
target_link_libraries(ArduCopter_libs AP_Winch) #stripped
target_link_libraries(ArduCopter_libs AP_LTM_Telem) #stripped
target_link_libraries(ArduCopter_libs AP_Devo_Telem) #stripped
target_link_libraries(ArduCopter_libs AC_AutoTune) #stripped
target_link_libraries(ArduCopter_libs AP_SurfaceDistance) #stripped
target_link_libraries(ArduCopter_libs AP_DroneCAN) #stripped
target_link_libraries(ArduCopter_libs AP_HAL_SITL) #stripped
target_link_libraries(ArduCopter_libs AP_CSVReader) #stripped
target_link_libraries(ArduCopter_libs SITL) #stripped
target_link_libraries(ArduCopter_libs AP_Scripting) #stripped
target_link_libraries(ArduCopter_libs AC_CustomControl) #stripped

#------------------------------------------------

set(bin_arducopter_SOURCES
    ArduCopter/AP_Arming_Copter.cpp
    ArduCopter/AP_ExternalControl_Copter.cpp
    ArduCopter/AP_Rally.cpp
    ArduCopter/AP_State.cpp
    ArduCopter/Attitude.cpp
    ArduCopter/Copter.cpp
    ArduCopter/GCS_Copter.cpp
    ArduCopter/GCS_MAVLink_Copter.cpp
    ArduCopter/Log.cpp
    ArduCopter/Parameters.cpp
    ArduCopter/RC_Channel_Copter.cpp
    ArduCopter/UserCode.cpp
    ArduCopter/UserParameters.cpp
    ArduCopter/afs_copter.cpp
    ArduCopter/autoyaw.cpp
    ArduCopter/avoidance.cpp
    ArduCopter/avoidance_adsb.cpp
    ArduCopter/baro_ground_effect.cpp
    ArduCopter/commands.cpp
    ArduCopter/compassmot.cpp
    ArduCopter/crash_check.cpp
    ArduCopter/ekf_check.cpp
    ArduCopter/esc_calibration.cpp
    ArduCopter/events.cpp
    ArduCopter/failsafe.cpp
    ArduCopter/fence.cpp
    ArduCopter/heli.cpp
    ArduCopter/inertia.cpp
    ArduCopter/land_detector.cpp
    ArduCopter/landing_gear.cpp
    ArduCopter/mode.cpp
    ArduCopter/mode_acro.cpp
    ArduCopter/mode_acro_heli.cpp
    ArduCopter/mode_althold.cpp
    ArduCopter/mode_auto.cpp
    ArduCopter/mode_autorotate.cpp
    ArduCopter/mode_autotune.cpp
    ArduCopter/mode_avoid_adsb.cpp
    ArduCopter/mode_brake.cpp
    ArduCopter/mode_circle.cpp
    ArduCopter/mode_drift.cpp
    ArduCopter/mode_flip.cpp
    ArduCopter/mode_flowhold.cpp
    ArduCopter/mode_follow.cpp
    ArduCopter/mode_guided.cpp
    ArduCopter/mode_guided_custom.cpp
    ArduCopter/mode_guided_nogps.cpp
    ArduCopter/mode_land.cpp
    ArduCopter/mode_loiter.cpp
    ArduCopter/mode_poshold.cpp
    ArduCopter/mode_rtl.cpp
    ArduCopter/mode_smart_rtl.cpp
    ArduCopter/mode_sport.cpp
    ArduCopter/mode_stabilize.cpp
    ArduCopter/mode_stabilize_heli.cpp
    ArduCopter/mode_systemid.cpp
    ArduCopter/mode_throw.cpp
    ArduCopter/mode_turtle.cpp
    ArduCopter/mode_zigzag.cpp
    ArduCopter/motor_test.cpp
    ArduCopter/motors.cpp
    ArduCopter/navigation.cpp
    ArduCopter/precision_landing.cpp
    ArduCopter/radio.cpp
    ArduCopter/rate_thread.cpp
    ArduCopter/sensors.cpp
    ArduCopter/standby.cpp
    ArduCopter/surface_tracking.cpp
    ArduCopter/system.cpp
    ArduCopter/takeoff.cpp
    ArduCopter/takeoff_check.cpp
    ArduCopter/terrain.cpp
    ArduCopter/toy_mode.cpp
    ArduCopter/tuning.cpp)

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

add_executable(bin_arducopter ${bin_arducopter_SOURCES})
#target_compile_definitions(bin_arducopter PRIVATE -DFRAME_CONFIG=MULTICOPTER_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
#target_include_directories(bin_arducopter PRIVATE ${bin_arducopter_INCLUDES})


target_link_libraries(bin_arducopter ArduCopter_libs) #clean

#------------------------------------------------

set(bin_arducopter-heli_SOURCES
    ArduCopter/AP_Arming_Copter.cpp
    ArduCopter/AP_ExternalControl_Copter.cpp
    ArduCopter/AP_Rally.cpp
    ArduCopter/AP_State.cpp
    ArduCopter/Attitude.cpp
    ArduCopter/Copter.cpp
    ArduCopter/GCS_Copter.cpp
    ArduCopter/GCS_MAVLink_Copter.cpp
    ArduCopter/Log.cpp
    ArduCopter/Parameters.cpp
    ArduCopter/RC_Channel_Copter.cpp
    ArduCopter/UserCode.cpp
    ArduCopter/UserParameters.cpp
    ArduCopter/afs_copter.cpp
    ArduCopter/autoyaw.cpp
    ArduCopter/avoidance.cpp
    ArduCopter/avoidance_adsb.cpp
    ArduCopter/baro_ground_effect.cpp
    ArduCopter/commands.cpp
    ArduCopter/compassmot.cpp
    ArduCopter/crash_check.cpp
    ArduCopter/ekf_check.cpp
    ArduCopter/esc_calibration.cpp
    ArduCopter/events.cpp
    ArduCopter/failsafe.cpp
    ArduCopter/fence.cpp
    ArduCopter/heli.cpp
    ArduCopter/inertia.cpp
    ArduCopter/land_detector.cpp
    ArduCopter/landing_gear.cpp
    ArduCopter/mode.cpp
    ArduCopter/mode_acro.cpp
    ArduCopter/mode_acro_heli.cpp
    ArduCopter/mode_althold.cpp
    ArduCopter/mode_auto.cpp
    ArduCopter/mode_autorotate.cpp
    ArduCopter/mode_autotune.cpp
    ArduCopter/mode_avoid_adsb.cpp
    ArduCopter/mode_brake.cpp
    ArduCopter/mode_circle.cpp
    ArduCopter/mode_drift.cpp
    ArduCopter/mode_flip.cpp
    ArduCopter/mode_flowhold.cpp
    ArduCopter/mode_follow.cpp
    ArduCopter/mode_guided.cpp
    ArduCopter/mode_guided_custom.cpp
    ArduCopter/mode_guided_nogps.cpp
    ArduCopter/mode_land.cpp
    ArduCopter/mode_loiter.cpp
    ArduCopter/mode_poshold.cpp
    ArduCopter/mode_rtl.cpp
    ArduCopter/mode_smart_rtl.cpp
    ArduCopter/mode_sport.cpp
    ArduCopter/mode_stabilize.cpp
    ArduCopter/mode_stabilize_heli.cpp
    ArduCopter/mode_systemid.cpp
    ArduCopter/mode_throw.cpp
    ArduCopter/mode_turtle.cpp
    ArduCopter/mode_zigzag.cpp
    ArduCopter/motor_test.cpp
    ArduCopter/motors.cpp
    ArduCopter/navigation.cpp
    ArduCopter/precision_landing.cpp
    ArduCopter/radio.cpp
    ArduCopter/rate_thread.cpp
    ArduCopter/sensors.cpp
    ArduCopter/standby.cpp
    ArduCopter/surface_tracking.cpp
    ArduCopter/system.cpp
    ArduCopter/takeoff.cpp
    ArduCopter/takeoff_check.cpp
    ArduCopter/terrain.cpp
    ArduCopter/toy_mode.cpp
    ArduCopter/tuning.cpp)

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

add_executable(bin_arducopter-heli ${bin_arducopter-heli_SOURCES})
#target_compile_definitions(bin_arducopter-heli PRIVATE -DFRAME_CONFIG=HELI_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
#target_include_directories(bin_arducopter-heli PRIVATE ${bin_arducopter-heli_INCLUDES})


target_link_libraries(bin_arducopter-heli ArduCopter_libs) #clean

#------------------------------------------------

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

add_library(ArduPlane_libs ${ArduPlane_libs_SOURCES}) #2
target_include_directories(ArduPlane_libs PRIVATE ${ArduPlane_libs_INCLUDES})

set_target_properties(ArduPlane_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(ArduPlane_libs AP_AccelCal) #stripped
target_link_libraries(ArduPlane_libs AP_ADC) #stripped
target_link_libraries(ArduPlane_libs AP_AHRS) #stripped
target_link_libraries(ArduPlane_libs AP_Airspeed) #stripped
target_link_libraries(ArduPlane_libs AP_Baro) #stripped
target_link_libraries(ArduPlane_libs AP_BattMonitor) #stripped
target_link_libraries(ArduPlane_libs AP_BoardConfig) #stripped
target_link_libraries(ArduPlane_libs AP_Camera) #stripped
target_link_libraries(ArduPlane_libs AP_Common) #stripped
target_link_libraries(ArduPlane_libs AP_Compass) #stripped
target_link_libraries(ArduPlane_libs AP_Declination) #stripped
target_link_libraries(ArduPlane_libs AP_GPS) #stripped
target_link_libraries(ArduPlane_libs AP_GSOF) #stripped
target_link_libraries(ArduPlane_libs AP_HAL) #stripped
target_link_libraries(ArduPlane_libs AP_HAL_Empty) #stripped
target_link_libraries(ArduPlane_libs AP_DDS) #stripped
target_link_libraries(ArduPlane_libs AP_InertialSensor) #stripped
target_link_libraries(ArduPlane_libs AP_Math) #stripped
target_link_libraries(ArduPlane_libs AP_Mission) #stripped
target_link_libraries(ArduPlane_libs AP_DAL) #stripped
target_link_libraries(ArduPlane_libs AP_NavEKF) #stripped
target_link_libraries(ArduPlane_libs AP_NavEKF2) #stripped
target_link_libraries(ArduPlane_libs AP_NavEKF3) #stripped
target_link_libraries(ArduPlane_libs AP_Notify) #stripped
target_link_libraries(ArduPlane_libs AP_OpticalFlow) #stripped
target_link_libraries(ArduPlane_libs AP_Param) #stripped
target_link_libraries(ArduPlane_libs AP_Rally) #stripped
target_link_libraries(ArduPlane_libs AP_LightWareSerial) #stripped
target_link_libraries(ArduPlane_libs AP_RangeFinder) #stripped
target_link_libraries(ArduPlane_libs AP_Scheduler) #stripped
target_link_libraries(ArduPlane_libs AP_SerialManager) #stripped
target_link_libraries(ArduPlane_libs AP_Terrain) #stripped
target_link_libraries(ArduPlane_libs AP_Vehicle) #stripped
target_link_libraries(ArduPlane_libs AP_InternalError) #stripped
target_link_libraries(ArduPlane_libs AP_Logger) #stripped
target_link_libraries(ArduPlane_libs Filter) #stripped
target_link_libraries(ArduPlane_libs GCS_MAVLink) #stripped
target_link_libraries(ArduPlane_libs RC_Channel) #stripped
target_link_libraries(ArduPlane_libs SRV_Channel) #stripped
target_link_libraries(ArduPlane_libs StorageManager) #stripped
target_link_libraries(ArduPlane_libs AP_Tuning) #stripped
target_link_libraries(ArduPlane_libs AP_RPM) #stripped
target_link_libraries(ArduPlane_libs AP_RSSI) #stripped
target_link_libraries(ArduPlane_libs AP_Mount) #stripped
target_link_libraries(ArduPlane_libs AP_Module) #stripped
target_link_libraries(ArduPlane_libs AP_Button) #stripped
target_link_libraries(ArduPlane_libs AP_ICEngine) #stripped
target_link_libraries(ArduPlane_libs AP_Networking) #stripped
target_link_libraries(ArduPlane_libs AP_Frsky_Telem) #stripped
target_link_libraries(ArduPlane_libs AP_IBus_Telem) #stripped
target_link_libraries(ArduPlane_libs AP_FlashStorage) #stripped
target_link_libraries(ArduPlane_libs AP_Relay) #stripped
target_link_libraries(ArduPlane_libs AP_ServoRelayEvents) #stripped
target_link_libraries(ArduPlane_libs AP_Volz_Protocol) #stripped
target_link_libraries(ArduPlane_libs AP_SBusOut) #stripped
target_link_libraries(ArduPlane_libs AP_IOMCU) #stripped
target_link_libraries(ArduPlane_libs AP_Parachute) #stripped
target_link_libraries(ArduPlane_libs AP_RAMTRON) #stripped
target_link_libraries(ArduPlane_libs AP_RCProtocol) #stripped
target_link_libraries(ArduPlane_libs AP_Radio) #stripped
target_link_libraries(ArduPlane_libs AP_TempCalibration) #stripped
target_link_libraries(ArduPlane_libs AP_VisualOdom) #stripped
target_link_libraries(ArduPlane_libs AP_BLHeli) #stripped
target_link_libraries(ArduPlane_libs AP_ROMFS) #stripped
target_link_libraries(ArduPlane_libs AP_Proximity) #stripped
target_link_libraries(ArduPlane_libs AP_Gripper) #stripped
target_link_libraries(ArduPlane_libs AP_RTC) #stripped
target_link_libraries(ArduPlane_libs AC_Sprayer) #stripped
target_link_libraries(ArduPlane_libs AC_Fence) #stripped
target_link_libraries(ArduPlane_libs AC_Avoidance) #stripped
target_link_libraries(ArduPlane_libs AP_LandingGear) #stripped
target_link_libraries(ArduPlane_libs AP_RobotisServo) #stripped
target_link_libraries(ArduPlane_libs AP_NMEA_Output) #stripped
target_link_libraries(ArduPlane_libs AP_OSD) #stripped
target_link_libraries(ArduPlane_libs AP_Filesystem) #stripped
target_link_libraries(ArduPlane_libs AP_ADSB) #stripped
target_link_libraries(ArduPlane_libs AC_PID) #stripped
target_link_libraries(ArduPlane_libs AP_SerialLED) #stripped
target_link_libraries(ArduPlane_libs AP_EFI) #stripped
target_link_libraries(ArduPlane_libs AP_Hott_Telem) #stripped
target_link_libraries(ArduPlane_libs AP_ESC_Telem) #stripped
target_link_libraries(ArduPlane_libs AP_Servo_Telem) #stripped
target_link_libraries(ArduPlane_libs AP_Stats) #stripped
target_link_libraries(ArduPlane_libs AP_GyroFFT) #stripped
target_link_libraries(ArduPlane_libs AP_RCTelemetry) #stripped
target_link_libraries(ArduPlane_libs AP_Generator) #stripped
target_link_libraries(ArduPlane_libs AP_MSP) #stripped
target_link_libraries(ArduPlane_libs AP_OLC) #stripped
target_link_libraries(ArduPlane_libs AP_WheelEncoder) #stripped
target_link_libraries(ArduPlane_libs AP_ExternalAHRS) #stripped
target_link_libraries(ArduPlane_libs AP_VideoTX) #stripped
target_link_libraries(ArduPlane_libs AP_FETtecOneWire) #stripped
target_link_libraries(ArduPlane_libs AP_TemperatureSensor) #stripped
target_link_libraries(ArduPlane_libs AP_Torqeedo) #stripped
target_link_libraries(ArduPlane_libs AP_CustomRotations) #stripped
target_link_libraries(ArduPlane_libs AP_AIS) #stripped
target_link_libraries(ArduPlane_libs AP_OpenDroneID) #stripped
target_link_libraries(ArduPlane_libs AP_CheckFirmware) #stripped
target_link_libraries(ArduPlane_libs AP_ExternalControl) #stripped
target_link_libraries(ArduPlane_libs AP_JSON) #stripped
target_link_libraries(ArduPlane_libs AP_Beacon) #stripped
target_link_libraries(ArduPlane_libs AP_Arming) #stripped
target_link_libraries(ArduPlane_libs AP_RCMapper) #stripped
target_link_libraries(ArduPlane_libs AP_MultiHeap) #stripped
target_link_libraries(ArduPlane_libs AP_Follow) #stripped
target_link_libraries(ArduPlane_libs AP_CANManager) #stripped
target_link_libraries(ArduPlane_libs AP_KDECAN) #stripped
target_link_libraries(ArduPlane_libs AP_PiccoloCAN) #stripped
target_link_libraries(ArduPlane_libs APM_Control) #stripped
target_link_libraries(ArduPlane_libs AP_AdvancedFailsafe) #stripped
target_link_libraries(ArduPlane_libs AP_Avoidance) #stripped
target_link_libraries(ArduPlane_libs AP_L1_Control) #stripped
target_link_libraries(ArduPlane_libs AP_Navigation) #stripped
target_link_libraries(ArduPlane_libs AP_TECS) #stripped
target_link_libraries(ArduPlane_libs AP_InertialNav) #stripped
target_link_libraries(ArduPlane_libs AC_WPNav) #stripped
target_link_libraries(ArduPlane_libs AC_AttitudeControl) #stripped
target_link_libraries(ArduPlane_libs AP_Motors) #stripped
target_link_libraries(ArduPlane_libs AP_Landing) #stripped
target_link_libraries(ArduPlane_libs PID) #stripped
target_link_libraries(ArduPlane_libs AP_Soaring) #stripped
target_link_libraries(ArduPlane_libs AP_LTM_Telem) #stripped
target_link_libraries(ArduPlane_libs AP_Devo_Telem) #stripped
target_link_libraries(ArduPlane_libs AC_AutoTune) #stripped
target_link_libraries(ArduPlane_libs AC_PrecLand) #stripped
target_link_libraries(ArduPlane_libs AP_IRLock) #stripped
target_link_libraries(ArduPlane_libs AP_Quicktune) #stripped
target_link_libraries(ArduPlane_libs AP_DroneCAN) #stripped
target_link_libraries(ArduPlane_libs AP_HAL_SITL) #stripped
target_link_libraries(ArduPlane_libs AP_CSVReader) #stripped
target_link_libraries(ArduPlane_libs SITL) #stripped
target_link_libraries(ArduPlane_libs AP_Scripting) #stripped
target_link_libraries(ArduPlane_libs AC_CustomControl) #stripped

#------------------------------------------------

set(bin_arduplane_SOURCES
    ArduPlane/AP_Arming_Plane.cpp
    ArduPlane/AP_ExternalControl_Plane.cpp
    ArduPlane/Attitude.cpp
    ArduPlane/GCS_MAVLink_Plane.cpp
    ArduPlane/GCS_Plane.cpp
    ArduPlane/Log.cpp
    ArduPlane/Parameters.cpp
    ArduPlane/Plane.cpp
    ArduPlane/RC_Channel_Plane.cpp
    ArduPlane/VTOL_Assist.cpp
    ArduPlane/afs_plane.cpp
    ArduPlane/altitude.cpp
    ArduPlane/avoidance_adsb.cpp
    ArduPlane/commands.cpp
    ArduPlane/commands_logic.cpp
    ArduPlane/control_modes.cpp
    ArduPlane/ekf_check.cpp
    ArduPlane/events.cpp
    ArduPlane/failsafe.cpp
    ArduPlane/fence.cpp
    ArduPlane/is_flying.cpp
    ArduPlane/mode.cpp
    ArduPlane/mode_LoiterAltQLand.cpp
    ArduPlane/mode_acro.cpp
    ArduPlane/mode_auto.cpp
    ArduPlane/mode_autoland.cpp
    ArduPlane/mode_autotune.cpp
    ArduPlane/mode_avoidADSB.cpp
    ArduPlane/mode_circle.cpp
    ArduPlane/mode_cruise.cpp
    ArduPlane/mode_fbwa.cpp
    ArduPlane/mode_fbwb.cpp
    ArduPlane/mode_guided.cpp
    ArduPlane/mode_loiter.cpp
    ArduPlane/mode_manual.cpp
    ArduPlane/mode_qacro.cpp
    ArduPlane/mode_qautotune.cpp
    ArduPlane/mode_qhover.cpp
    ArduPlane/mode_qland.cpp
    ArduPlane/mode_qloiter.cpp
    ArduPlane/mode_qrtl.cpp
    ArduPlane/mode_qstabilize.cpp
    ArduPlane/mode_rtl.cpp
    ArduPlane/mode_stabilize.cpp
    ArduPlane/mode_takeoff.cpp
    ArduPlane/mode_thermal.cpp
    ArduPlane/mode_training.cpp
    ArduPlane/motor_test.cpp
    ArduPlane/navigation.cpp
    ArduPlane/parachute.cpp
    ArduPlane/pullup.cpp
    ArduPlane/qautotune.cpp
    ArduPlane/quadplane.cpp
    ArduPlane/radio.cpp
    ArduPlane/reverse_thrust.cpp
    ArduPlane/sensors.cpp
    ArduPlane/servos.cpp
    ArduPlane/soaring.cpp
    ArduPlane/system.cpp
    ArduPlane/systemid.cpp
    ArduPlane/tailsitter.cpp
    ArduPlane/takeoff.cpp
    ArduPlane/tiltrotor.cpp
    ArduPlane/tuning.cpp)

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

add_executable(bin_arduplane ${bin_arduplane_SOURCES})
#target_compile_definitions(bin_arduplane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
#target_include_directories(bin_arduplane PRIVATE ${bin_arduplane_INCLUDES})


target_link_libraries(bin_arduplane ArduPlane_libs) #clean

#------------------------------------------------

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

add_library(ArduSub_libs ${ArduSub_libs_SOURCES}) #2
target_include_directories(ArduSub_libs PRIVATE ${ArduSub_libs_INCLUDES})

set_target_properties(ArduSub_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(ArduSub_libs AP_AccelCal) #stripped
target_link_libraries(ArduSub_libs AP_ADC) #stripped
target_link_libraries(ArduSub_libs AP_AHRS) #stripped
target_link_libraries(ArduSub_libs AP_Airspeed) #stripped
target_link_libraries(ArduSub_libs AP_Baro) #stripped
target_link_libraries(ArduSub_libs AP_BattMonitor) #stripped
target_link_libraries(ArduSub_libs AP_BoardConfig) #stripped
target_link_libraries(ArduSub_libs AP_Camera) #stripped
target_link_libraries(ArduSub_libs AP_Common) #stripped
target_link_libraries(ArduSub_libs AP_Compass) #stripped
target_link_libraries(ArduSub_libs AP_Declination) #stripped
target_link_libraries(ArduSub_libs AP_GPS) #stripped
target_link_libraries(ArduSub_libs AP_GSOF) #stripped
target_link_libraries(ArduSub_libs AP_HAL) #stripped
target_link_libraries(ArduSub_libs AP_HAL_Empty) #stripped
target_link_libraries(ArduSub_libs AP_DDS) #stripped
target_link_libraries(ArduSub_libs AP_InertialSensor) #stripped
target_link_libraries(ArduSub_libs AP_Math) #stripped
target_link_libraries(ArduSub_libs AP_Mission) #stripped
target_link_libraries(ArduSub_libs AP_DAL) #stripped
target_link_libraries(ArduSub_libs AP_NavEKF) #stripped
target_link_libraries(ArduSub_libs AP_NavEKF2) #stripped
target_link_libraries(ArduSub_libs AP_NavEKF3) #stripped
target_link_libraries(ArduSub_libs AP_Notify) #stripped
target_link_libraries(ArduSub_libs AP_OpticalFlow) #stripped
target_link_libraries(ArduSub_libs AP_Param) #stripped
target_link_libraries(ArduSub_libs AP_Rally) #stripped
target_link_libraries(ArduSub_libs AP_LightWareSerial) #stripped
target_link_libraries(ArduSub_libs AP_RangeFinder) #stripped
target_link_libraries(ArduSub_libs AP_Scheduler) #stripped
target_link_libraries(ArduSub_libs AP_SerialManager) #stripped
target_link_libraries(ArduSub_libs AP_Terrain) #stripped
target_link_libraries(ArduSub_libs AP_Vehicle) #stripped
target_link_libraries(ArduSub_libs AP_InternalError) #stripped
target_link_libraries(ArduSub_libs AP_Logger) #stripped
target_link_libraries(ArduSub_libs Filter) #stripped
target_link_libraries(ArduSub_libs GCS_MAVLink) #stripped
target_link_libraries(ArduSub_libs RC_Channel) #stripped
target_link_libraries(ArduSub_libs SRV_Channel) #stripped
target_link_libraries(ArduSub_libs StorageManager) #stripped
target_link_libraries(ArduSub_libs AP_Tuning) #stripped
target_link_libraries(ArduSub_libs AP_RPM) #stripped
target_link_libraries(ArduSub_libs AP_RSSI) #stripped
target_link_libraries(ArduSub_libs AP_Mount) #stripped
target_link_libraries(ArduSub_libs AP_Module) #stripped
target_link_libraries(ArduSub_libs AP_Button) #stripped
target_link_libraries(ArduSub_libs AP_ICEngine) #stripped
target_link_libraries(ArduSub_libs AP_Networking) #stripped
target_link_libraries(ArduSub_libs AP_Frsky_Telem) #stripped
target_link_libraries(ArduSub_libs AP_IBus_Telem) #stripped
target_link_libraries(ArduSub_libs AP_FlashStorage) #stripped
target_link_libraries(ArduSub_libs AP_Relay) #stripped
target_link_libraries(ArduSub_libs AP_ServoRelayEvents) #stripped
target_link_libraries(ArduSub_libs AP_Volz_Protocol) #stripped
target_link_libraries(ArduSub_libs AP_SBusOut) #stripped
target_link_libraries(ArduSub_libs AP_IOMCU) #stripped
target_link_libraries(ArduSub_libs AP_Parachute) #stripped
target_link_libraries(ArduSub_libs AP_RAMTRON) #stripped
target_link_libraries(ArduSub_libs AP_RCProtocol) #stripped
target_link_libraries(ArduSub_libs AP_Radio) #stripped
target_link_libraries(ArduSub_libs AP_TempCalibration) #stripped
target_link_libraries(ArduSub_libs AP_VisualOdom) #stripped
target_link_libraries(ArduSub_libs AP_BLHeli) #stripped
target_link_libraries(ArduSub_libs AP_ROMFS) #stripped
target_link_libraries(ArduSub_libs AP_Proximity) #stripped
target_link_libraries(ArduSub_libs AP_Gripper) #stripped
target_link_libraries(ArduSub_libs AP_RTC) #stripped
target_link_libraries(ArduSub_libs AC_Sprayer) #stripped
target_link_libraries(ArduSub_libs AC_Fence) #stripped
target_link_libraries(ArduSub_libs AC_Avoidance) #stripped
target_link_libraries(ArduSub_libs AP_LandingGear) #stripped
target_link_libraries(ArduSub_libs AP_RobotisServo) #stripped
target_link_libraries(ArduSub_libs AP_NMEA_Output) #stripped
target_link_libraries(ArduSub_libs AP_OSD) #stripped
target_link_libraries(ArduSub_libs AP_Filesystem) #stripped
target_link_libraries(ArduSub_libs AP_ADSB) #stripped
target_link_libraries(ArduSub_libs AC_PID) #stripped
target_link_libraries(ArduSub_libs AP_SerialLED) #stripped
target_link_libraries(ArduSub_libs AP_EFI) #stripped
target_link_libraries(ArduSub_libs AP_Hott_Telem) #stripped
target_link_libraries(ArduSub_libs AP_ESC_Telem) #stripped
target_link_libraries(ArduSub_libs AP_Servo_Telem) #stripped
target_link_libraries(ArduSub_libs AP_Stats) #stripped
target_link_libraries(ArduSub_libs AP_GyroFFT) #stripped
target_link_libraries(ArduSub_libs AP_RCTelemetry) #stripped
target_link_libraries(ArduSub_libs AP_Generator) #stripped
target_link_libraries(ArduSub_libs AP_MSP) #stripped
target_link_libraries(ArduSub_libs AP_OLC) #stripped
target_link_libraries(ArduSub_libs AP_WheelEncoder) #stripped
target_link_libraries(ArduSub_libs AP_ExternalAHRS) #stripped
target_link_libraries(ArduSub_libs AP_VideoTX) #stripped
target_link_libraries(ArduSub_libs AP_FETtecOneWire) #stripped
target_link_libraries(ArduSub_libs AP_TemperatureSensor) #stripped
target_link_libraries(ArduSub_libs AP_Torqeedo) #stripped
target_link_libraries(ArduSub_libs AP_CustomRotations) #stripped
target_link_libraries(ArduSub_libs AP_AIS) #stripped
target_link_libraries(ArduSub_libs AP_OpenDroneID) #stripped
target_link_libraries(ArduSub_libs AP_CheckFirmware) #stripped
target_link_libraries(ArduSub_libs AP_ExternalControl) #stripped
target_link_libraries(ArduSub_libs AP_JSON) #stripped
target_link_libraries(ArduSub_libs AP_Beacon) #stripped
target_link_libraries(ArduSub_libs AP_Arming) #stripped
target_link_libraries(ArduSub_libs AP_RCMapper) #stripped
target_link_libraries(ArduSub_libs AP_MultiHeap) #stripped
target_link_libraries(ArduSub_libs AP_Follow) #stripped
target_link_libraries(ArduSub_libs AP_CANManager) #stripped
target_link_libraries(ArduSub_libs AP_KDECAN) #stripped
target_link_libraries(ArduSub_libs AP_PiccoloCAN) #stripped
target_link_libraries(ArduSub_libs AC_AttitudeControl) #stripped
target_link_libraries(ArduSub_libs AC_WPNav) #stripped
target_link_libraries(ArduSub_libs AP_InertialNav) #stripped
target_link_libraries(ArduSub_libs AP_JSButton) #stripped
target_link_libraries(ArduSub_libs AP_LeakDetector) #stripped
target_link_libraries(ArduSub_libs AP_Motors) #stripped
target_link_libraries(ArduSub_libs AP_DroneCAN) #stripped
target_link_libraries(ArduSub_libs AP_HAL_SITL) #stripped
target_link_libraries(ArduSub_libs AP_CSVReader) #stripped
target_link_libraries(ArduSub_libs SITL) #stripped
target_link_libraries(ArduSub_libs AP_Scripting) #stripped
target_link_libraries(ArduSub_libs AC_CustomControl) #stripped

#------------------------------------------------

set(bin_ardusub_SOURCES
    ArduSub/AP_Arming_Sub.cpp
    ArduSub/AP_State.cpp
    ArduSub/Attitude.cpp
    ArduSub/GCS_MAVLink_Sub.cpp
    ArduSub/GCS_Sub.cpp
    ArduSub/Log.cpp
    ArduSub/Parameters.cpp
    ArduSub/RC_Channel_Sub.cpp
    ArduSub/Sub.cpp
    ArduSub/UserCode.cpp
    ArduSub/actuators.cpp
    ArduSub/commands.cpp
    ArduSub/commands_logic.cpp
    ArduSub/failsafe.cpp
    ArduSub/fence.cpp
    ArduSub/inertia.cpp
    ArduSub/joystick.cpp
    ArduSub/mode.cpp
    ArduSub/mode_acro.cpp
    ArduSub/mode_althold.cpp
    ArduSub/mode_auto.cpp
    ArduSub/mode_circle.cpp
    ArduSub/mode_guided.cpp
    ArduSub/mode_manual.cpp
    ArduSub/mode_motordetect.cpp
    ArduSub/mode_poshold.cpp
    ArduSub/mode_stabilize.cpp
    ArduSub/mode_surface.cpp
    ArduSub/mode_surftrak.cpp
    ArduSub/motors.cpp
    ArduSub/radio.cpp
    ArduSub/script_button.cpp
    ArduSub/sensors.cpp
    ArduSub/surface_bottom_detector.cpp
    ArduSub/system.cpp
    ArduSub/terrain.cpp
    ArduSub/turn_counter.cpp)

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

add_executable(bin_ardusub ${bin_ardusub_SOURCES})
#target_compile_definitions(bin_ardusub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
#target_include_directories(bin_ardusub PRIVATE ${bin_ardusub_INCLUDES})


target_link_libraries(bin_ardusub ArduSub_libs) #clean

#------------------------------------------------

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

add_library(Blimp_libs ${Blimp_libs_SOURCES}) #2
target_include_directories(Blimp_libs PRIVATE ${Blimp_libs_INCLUDES})

set_target_properties(Blimp_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(Blimp_libs AP_AccelCal) #stripped
target_link_libraries(Blimp_libs AP_ADC) #stripped
target_link_libraries(Blimp_libs AP_AHRS) #stripped
target_link_libraries(Blimp_libs AP_Airspeed) #stripped
target_link_libraries(Blimp_libs AP_Baro) #stripped
target_link_libraries(Blimp_libs AP_BattMonitor) #stripped
target_link_libraries(Blimp_libs AP_BoardConfig) #stripped
target_link_libraries(Blimp_libs AP_Camera) #stripped
target_link_libraries(Blimp_libs AP_Common) #stripped
target_link_libraries(Blimp_libs AP_Compass) #stripped
target_link_libraries(Blimp_libs AP_Declination) #stripped
target_link_libraries(Blimp_libs AP_GPS) #stripped
target_link_libraries(Blimp_libs AP_GSOF) #stripped
target_link_libraries(Blimp_libs AP_HAL) #stripped
target_link_libraries(Blimp_libs AP_HAL_Empty) #stripped
target_link_libraries(Blimp_libs AP_DDS) #stripped
target_link_libraries(Blimp_libs AP_InertialSensor) #stripped
target_link_libraries(Blimp_libs AP_Math) #stripped
target_link_libraries(Blimp_libs AP_Mission) #stripped
target_link_libraries(Blimp_libs AP_DAL) #stripped
target_link_libraries(Blimp_libs AP_NavEKF) #stripped
target_link_libraries(Blimp_libs AP_NavEKF2) #stripped
target_link_libraries(Blimp_libs AP_NavEKF3) #stripped
target_link_libraries(Blimp_libs AP_Notify) #stripped
target_link_libraries(Blimp_libs AP_OpticalFlow) #stripped
target_link_libraries(Blimp_libs AP_Param) #stripped
target_link_libraries(Blimp_libs AP_Rally) #stripped
target_link_libraries(Blimp_libs AP_LightWareSerial) #stripped
target_link_libraries(Blimp_libs AP_RangeFinder) #stripped
target_link_libraries(Blimp_libs AP_Scheduler) #stripped
target_link_libraries(Blimp_libs AP_SerialManager) #stripped
target_link_libraries(Blimp_libs AP_Terrain) #stripped
target_link_libraries(Blimp_libs AP_Vehicle) #stripped
target_link_libraries(Blimp_libs AP_InternalError) #stripped
target_link_libraries(Blimp_libs AP_Logger) #stripped
target_link_libraries(Blimp_libs Filter) #stripped
target_link_libraries(Blimp_libs GCS_MAVLink) #stripped
target_link_libraries(Blimp_libs RC_Channel) #stripped
target_link_libraries(Blimp_libs SRV_Channel) #stripped
target_link_libraries(Blimp_libs StorageManager) #stripped
target_link_libraries(Blimp_libs AP_Tuning) #stripped
target_link_libraries(Blimp_libs AP_RPM) #stripped
target_link_libraries(Blimp_libs AP_RSSI) #stripped
target_link_libraries(Blimp_libs AP_Mount) #stripped
target_link_libraries(Blimp_libs AP_Module) #stripped
target_link_libraries(Blimp_libs AP_Button) #stripped
target_link_libraries(Blimp_libs AP_ICEngine) #stripped
target_link_libraries(Blimp_libs AP_Networking) #stripped
target_link_libraries(Blimp_libs AP_Frsky_Telem) #stripped
target_link_libraries(Blimp_libs AP_IBus_Telem) #stripped
target_link_libraries(Blimp_libs AP_FlashStorage) #stripped
target_link_libraries(Blimp_libs AP_Relay) #stripped
target_link_libraries(Blimp_libs AP_ServoRelayEvents) #stripped
target_link_libraries(Blimp_libs AP_Volz_Protocol) #stripped
target_link_libraries(Blimp_libs AP_SBusOut) #stripped
target_link_libraries(Blimp_libs AP_IOMCU) #stripped
target_link_libraries(Blimp_libs AP_Parachute) #stripped
target_link_libraries(Blimp_libs AP_RAMTRON) #stripped
target_link_libraries(Blimp_libs AP_RCProtocol) #stripped
target_link_libraries(Blimp_libs AP_Radio) #stripped
target_link_libraries(Blimp_libs AP_TempCalibration) #stripped
target_link_libraries(Blimp_libs AP_VisualOdom) #stripped
target_link_libraries(Blimp_libs AP_BLHeli) #stripped
target_link_libraries(Blimp_libs AP_ROMFS) #stripped
target_link_libraries(Blimp_libs AP_Proximity) #stripped
target_link_libraries(Blimp_libs AP_Gripper) #stripped
target_link_libraries(Blimp_libs AP_RTC) #stripped
target_link_libraries(Blimp_libs AC_Sprayer) #stripped
target_link_libraries(Blimp_libs AC_Fence) #stripped
target_link_libraries(Blimp_libs AC_Avoidance) #stripped
target_link_libraries(Blimp_libs AP_LandingGear) #stripped
target_link_libraries(Blimp_libs AP_RobotisServo) #stripped
target_link_libraries(Blimp_libs AP_NMEA_Output) #stripped
target_link_libraries(Blimp_libs AP_OSD) #stripped
target_link_libraries(Blimp_libs AP_Filesystem) #stripped
target_link_libraries(Blimp_libs AP_ADSB) #stripped
target_link_libraries(Blimp_libs AC_PID) #stripped
target_link_libraries(Blimp_libs AP_SerialLED) #stripped
target_link_libraries(Blimp_libs AP_EFI) #stripped
target_link_libraries(Blimp_libs AP_Hott_Telem) #stripped
target_link_libraries(Blimp_libs AP_ESC_Telem) #stripped
target_link_libraries(Blimp_libs AP_Servo_Telem) #stripped
target_link_libraries(Blimp_libs AP_Stats) #stripped
target_link_libraries(Blimp_libs AP_GyroFFT) #stripped
target_link_libraries(Blimp_libs AP_RCTelemetry) #stripped
target_link_libraries(Blimp_libs AP_Generator) #stripped
target_link_libraries(Blimp_libs AP_MSP) #stripped
target_link_libraries(Blimp_libs AP_OLC) #stripped
target_link_libraries(Blimp_libs AP_WheelEncoder) #stripped
target_link_libraries(Blimp_libs AP_ExternalAHRS) #stripped
target_link_libraries(Blimp_libs AP_VideoTX) #stripped
target_link_libraries(Blimp_libs AP_FETtecOneWire) #stripped
target_link_libraries(Blimp_libs AP_TemperatureSensor) #stripped
target_link_libraries(Blimp_libs AP_Torqeedo) #stripped
target_link_libraries(Blimp_libs AP_CustomRotations) #stripped
target_link_libraries(Blimp_libs AP_AIS) #stripped
target_link_libraries(Blimp_libs AP_OpenDroneID) #stripped
target_link_libraries(Blimp_libs AP_CheckFirmware) #stripped
target_link_libraries(Blimp_libs AP_ExternalControl) #stripped
target_link_libraries(Blimp_libs AP_JSON) #stripped
target_link_libraries(Blimp_libs AP_Beacon) #stripped
target_link_libraries(Blimp_libs AP_Arming) #stripped
target_link_libraries(Blimp_libs AP_RCMapper) #stripped
target_link_libraries(Blimp_libs AP_MultiHeap) #stripped
target_link_libraries(Blimp_libs AP_Follow) #stripped
target_link_libraries(Blimp_libs AP_CANManager) #stripped
target_link_libraries(Blimp_libs AP_KDECAN) #stripped
target_link_libraries(Blimp_libs AP_PiccoloCAN) #stripped
target_link_libraries(Blimp_libs AC_InputManager) #stripped
target_link_libraries(Blimp_libs AP_InertialNav) #stripped
target_link_libraries(Blimp_libs AP_Avoidance) #stripped
target_link_libraries(Blimp_libs AP_LTM_Telem) #stripped
target_link_libraries(Blimp_libs AP_Devo_Telem) #stripped
target_link_libraries(Blimp_libs AP_AdvancedFailsafe) #stripped
target_link_libraries(Blimp_libs AC_AttitudeControl) #stripped
target_link_libraries(Blimp_libs AP_DroneCAN) #stripped
target_link_libraries(Blimp_libs AP_HAL_SITL) #stripped
target_link_libraries(Blimp_libs AP_CSVReader) #stripped
target_link_libraries(Blimp_libs SITL) #stripped
target_link_libraries(Blimp_libs AP_Scripting) #stripped
target_link_libraries(Blimp_libs AC_CustomControl) #stripped

#------------------------------------------------

set(bin_blimp_SOURCES
    Blimp/AP_Arming_Blimp.cpp
    Blimp/AP_State.cpp
    Blimp/Blimp.cpp
    Blimp/Fins.cpp
    Blimp/GCS_Blimp.cpp
    Blimp/GCS_MAVLink_Blimp.cpp
    Blimp/Log.cpp
    Blimp/Loiter.cpp
    Blimp/Parameters.cpp
    Blimp/RC_Channel_Blimp.cpp
    Blimp/commands.cpp
    Blimp/ekf_check.cpp
    Blimp/events.cpp
    Blimp/failsafe.cpp
    Blimp/inertia.cpp
    Blimp/mode.cpp
    Blimp/mode_land.cpp
    Blimp/mode_loiter.cpp
    Blimp/mode_manual.cpp
    Blimp/mode_rtl.cpp
    Blimp/mode_velocity.cpp
    Blimp/motors.cpp
    Blimp/radio.cpp
    Blimp/sensors.cpp
    Blimp/system.cpp)

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

add_executable(bin_blimp ${bin_blimp_SOURCES})
#target_compile_definitions(bin_blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
#target_include_directories(bin_blimp PRIVATE ${bin_blimp_INCLUDES})


target_link_libraries(bin_blimp Blimp_libs) #clean

#------------------------------------------------

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

add_library(Rover_libs ${Rover_libs_SOURCES}) #2
target_include_directories(Rover_libs PRIVATE ${Rover_libs_INCLUDES})

set_target_properties(Rover_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(Rover_libs AP_AccelCal) #stripped
target_link_libraries(Rover_libs AP_ADC) #stripped
target_link_libraries(Rover_libs AP_AHRS) #stripped
target_link_libraries(Rover_libs AP_Airspeed) #stripped
target_link_libraries(Rover_libs AP_Baro) #stripped
target_link_libraries(Rover_libs AP_BattMonitor) #stripped
target_link_libraries(Rover_libs AP_BoardConfig) #stripped
target_link_libraries(Rover_libs AP_Camera) #stripped
target_link_libraries(Rover_libs AP_Common) #stripped
target_link_libraries(Rover_libs AP_Compass) #stripped
target_link_libraries(Rover_libs AP_Declination) #stripped
target_link_libraries(Rover_libs AP_GPS) #stripped
target_link_libraries(Rover_libs AP_GSOF) #stripped
target_link_libraries(Rover_libs AP_HAL) #stripped
target_link_libraries(Rover_libs AP_HAL_Empty) #stripped
target_link_libraries(Rover_libs AP_DDS) #stripped
target_link_libraries(Rover_libs AP_InertialSensor) #stripped
target_link_libraries(Rover_libs AP_Math) #stripped
target_link_libraries(Rover_libs AP_Mission) #stripped
target_link_libraries(Rover_libs AP_DAL) #stripped
target_link_libraries(Rover_libs AP_NavEKF) #stripped
target_link_libraries(Rover_libs AP_NavEKF2) #stripped
target_link_libraries(Rover_libs AP_NavEKF3) #stripped
target_link_libraries(Rover_libs AP_Notify) #stripped
target_link_libraries(Rover_libs AP_OpticalFlow) #stripped
target_link_libraries(Rover_libs AP_Param) #stripped
target_link_libraries(Rover_libs AP_Rally) #stripped
target_link_libraries(Rover_libs AP_LightWareSerial) #stripped
target_link_libraries(Rover_libs AP_RangeFinder) #stripped
target_link_libraries(Rover_libs AP_Scheduler) #stripped
target_link_libraries(Rover_libs AP_SerialManager) #stripped
target_link_libraries(Rover_libs AP_Terrain) #stripped
target_link_libraries(Rover_libs AP_Vehicle) #stripped
target_link_libraries(Rover_libs AP_InternalError) #stripped
target_link_libraries(Rover_libs AP_Logger) #stripped
target_link_libraries(Rover_libs Filter) #stripped
target_link_libraries(Rover_libs GCS_MAVLink) #stripped
target_link_libraries(Rover_libs RC_Channel) #stripped
target_link_libraries(Rover_libs SRV_Channel) #stripped
target_link_libraries(Rover_libs StorageManager) #stripped
target_link_libraries(Rover_libs AP_Tuning) #stripped
target_link_libraries(Rover_libs AP_RPM) #stripped
target_link_libraries(Rover_libs AP_RSSI) #stripped
target_link_libraries(Rover_libs AP_Mount) #stripped
target_link_libraries(Rover_libs AP_Module) #stripped
target_link_libraries(Rover_libs AP_Button) #stripped
target_link_libraries(Rover_libs AP_ICEngine) #stripped
target_link_libraries(Rover_libs AP_Networking) #stripped
target_link_libraries(Rover_libs AP_Frsky_Telem) #stripped
target_link_libraries(Rover_libs AP_IBus_Telem) #stripped
target_link_libraries(Rover_libs AP_FlashStorage) #stripped
target_link_libraries(Rover_libs AP_Relay) #stripped
target_link_libraries(Rover_libs AP_ServoRelayEvents) #stripped
target_link_libraries(Rover_libs AP_Volz_Protocol) #stripped
target_link_libraries(Rover_libs AP_SBusOut) #stripped
target_link_libraries(Rover_libs AP_IOMCU) #stripped
target_link_libraries(Rover_libs AP_Parachute) #stripped
target_link_libraries(Rover_libs AP_RAMTRON) #stripped
target_link_libraries(Rover_libs AP_RCProtocol) #stripped
target_link_libraries(Rover_libs AP_Radio) #stripped
target_link_libraries(Rover_libs AP_TempCalibration) #stripped
target_link_libraries(Rover_libs AP_VisualOdom) #stripped
target_link_libraries(Rover_libs AP_BLHeli) #stripped
target_link_libraries(Rover_libs AP_ROMFS) #stripped
target_link_libraries(Rover_libs AP_Proximity) #stripped
target_link_libraries(Rover_libs AP_Gripper) #stripped
target_link_libraries(Rover_libs AP_RTC) #stripped
target_link_libraries(Rover_libs AC_Sprayer) #stripped
target_link_libraries(Rover_libs AC_Fence) #stripped
target_link_libraries(Rover_libs AC_Avoidance) #stripped
target_link_libraries(Rover_libs AP_LandingGear) #stripped
target_link_libraries(Rover_libs AP_RobotisServo) #stripped
target_link_libraries(Rover_libs AP_NMEA_Output) #stripped
target_link_libraries(Rover_libs AP_OSD) #stripped
target_link_libraries(Rover_libs AP_Filesystem) #stripped
target_link_libraries(Rover_libs AP_ADSB) #stripped
target_link_libraries(Rover_libs AC_PID) #stripped
target_link_libraries(Rover_libs AP_SerialLED) #stripped
target_link_libraries(Rover_libs AP_EFI) #stripped
target_link_libraries(Rover_libs AP_Hott_Telem) #stripped
target_link_libraries(Rover_libs AP_ESC_Telem) #stripped
target_link_libraries(Rover_libs AP_Servo_Telem) #stripped
target_link_libraries(Rover_libs AP_Stats) #stripped
target_link_libraries(Rover_libs AP_GyroFFT) #stripped
target_link_libraries(Rover_libs AP_RCTelemetry) #stripped
target_link_libraries(Rover_libs AP_Generator) #stripped
target_link_libraries(Rover_libs AP_MSP) #stripped
target_link_libraries(Rover_libs AP_OLC) #stripped
target_link_libraries(Rover_libs AP_WheelEncoder) #stripped
target_link_libraries(Rover_libs AP_ExternalAHRS) #stripped
target_link_libraries(Rover_libs AP_VideoTX) #stripped
target_link_libraries(Rover_libs AP_FETtecOneWire) #stripped
target_link_libraries(Rover_libs AP_TemperatureSensor) #stripped
target_link_libraries(Rover_libs AP_Torqeedo) #stripped
target_link_libraries(Rover_libs AP_CustomRotations) #stripped
target_link_libraries(Rover_libs AP_AIS) #stripped
target_link_libraries(Rover_libs AP_OpenDroneID) #stripped
target_link_libraries(Rover_libs AP_CheckFirmware) #stripped
target_link_libraries(Rover_libs AP_ExternalControl) #stripped
target_link_libraries(Rover_libs AP_JSON) #stripped
target_link_libraries(Rover_libs AP_Beacon) #stripped
target_link_libraries(Rover_libs AP_Arming) #stripped
target_link_libraries(Rover_libs AP_RCMapper) #stripped
target_link_libraries(Rover_libs AP_MultiHeap) #stripped
target_link_libraries(Rover_libs AP_Follow) #stripped
target_link_libraries(Rover_libs AP_CANManager) #stripped
target_link_libraries(Rover_libs AP_KDECAN) #stripped
target_link_libraries(Rover_libs AP_PiccoloCAN) #stripped
target_link_libraries(Rover_libs APM_Control) #stripped
target_link_libraries(Rover_libs AP_Navigation) #stripped
target_link_libraries(Rover_libs AR_WPNav) #stripped
target_link_libraries(Rover_libs AP_AdvancedFailsafe) #stripped
target_link_libraries(Rover_libs AP_SmartRTL) #stripped
target_link_libraries(Rover_libs AC_AttitudeControl) #stripped
target_link_libraries(Rover_libs AP_LTM_Telem) #stripped
target_link_libraries(Rover_libs AP_Devo_Telem) #stripped
target_link_libraries(Rover_libs AP_WindVane) #stripped
target_link_libraries(Rover_libs AR_Motors) #stripped
target_link_libraries(Rover_libs AC_PrecLand) #stripped
target_link_libraries(Rover_libs AP_IRLock) #stripped
target_link_libraries(Rover_libs AP_DroneCAN) #stripped
target_link_libraries(Rover_libs AP_HAL_SITL) #stripped
target_link_libraries(Rover_libs AP_CSVReader) #stripped
target_link_libraries(Rover_libs SITL) #stripped
target_link_libraries(Rover_libs AP_Scripting) #stripped
target_link_libraries(Rover_libs AC_CustomControl) #stripped

#------------------------------------------------

set(bin_ardurover_SOURCES
    Rover/AP_Arming_Rover.cpp
    Rover/AP_ExternalControl_Rover.cpp
    Rover/AP_Rally.cpp
    Rover/GCS_MAVLink_Rover.cpp
    Rover/GCS_Rover.cpp
    Rover/Log.cpp
    Rover/Parameters.cpp
    Rover/RC_Channel_Rover.cpp
    Rover/Rover.cpp
    Rover/Steering.cpp
    Rover/afs_rover.cpp
    Rover/balance_bot.cpp
    Rover/commands.cpp
    Rover/crash_check.cpp
    Rover/cruise_learn.cpp
    Rover/ekf_check.cpp
    Rover/failsafe.cpp
    Rover/fence.cpp
    Rover/mode.cpp
    Rover/mode_acro.cpp
    Rover/mode_auto.cpp
    Rover/mode_circle.cpp
    Rover/mode_dock.cpp
    Rover/mode_follow.cpp
    Rover/mode_guided.cpp
    Rover/mode_hold.cpp
    Rover/mode_loiter.cpp
    Rover/mode_manual.cpp
    Rover/mode_rtl.cpp
    Rover/mode_simple.cpp
    Rover/mode_smart_rtl.cpp
    Rover/mode_steering.cpp
    Rover/motor_test.cpp
    Rover/precision_landing.cpp
    Rover/radio.cpp
    Rover/sailboat.cpp
    Rover/sensors.cpp
    Rover/system.cpp)

set(bin_ardurover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(bin_ardurover ${bin_ardurover_SOURCES})
#target_compile_definitions(bin_ardurover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
#target_include_directories(bin_ardurover PRIVATE ${bin_ardurover_INCLUDES})


target_link_libraries(bin_ardurover Rover_libs) #clean

