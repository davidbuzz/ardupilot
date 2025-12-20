cmake_minimum_required (VERSION 3.13.0)
project (ArduPilot)

include_directories(.)
include_directories(..)
include_directories(../..)
include_directories(../../..)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_definitions(-DAP_BARO_PROBE_EXTERNAL_I2C_BUSES=1 -DAP_CUSTOMCONTROL_ENABLED=1 -DAP_DDS_ENABLED=0 -DAP_SCRIPTING_CHECKS=1 -DCANARD_ALLOCATE_SEM=1 -DCANARD_ENABLE_ASSERTS=1 -DCANARD_ENABLE_CANFD=1 -DCANARD_ENABLE_DEADLINE=1 -DCANARD_IFACE_ALL=3 -DCANARD_MULTI_IFACE=1 -DCONFIG_HAL_BOARD=HAL_BOARD_SITL -DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE -DDRONECAN_CXX_WRAPPERS=1 -DENABLE_ONVIF=0 -DLUA_32BITS=1 -DUSE_USER_HELPERS=1) #1

set(CMAKE_C_FLAGS "-ffunction-sections -fdata-sections -fsigned-char -Wall -Wextra -Werror=format -Wpointer-arith -Wcast-align -Wno-missing-field-initializers -Wno-unused-parameter -Wno-redundant-decls -Wno-unknown-pragmas -Wno-trigraphs -Werror=return-type -Werror=unused-result -Werror=unused-variable -Werror=narrowing -Werror=attributes -Werror=overflow -Werror=parentheses -Werror=format-extra-args -Werror=ignored-qualifiers -Werror=undef -DARDUPILOT_BUILD -Wno-format-contains-nul -fsingle-precision-constant -Werror=use-after-free -DCANARD_64_BIT=1 -MMD -ffile-prefix-map=../..=../..")
set(CMAKE_CXX_FLAGS "-std=gnu++11 -fdata-sections -ffunction-sections -fno-exceptions -fsigned-char -Wall -Wextra -Wpointer-arith -Wno-unused-parameter -Wno-missing-field-initializers -Wno-redundant-decls -Wno-unknown-pragmas -Wno-expansion-to-defined -Werror=reorder -Werror=cast-align -Werror=attributes -Werror=format-security -Werror=format-extra-args -Werror=enum-compare -Werror=format -Werror=array-bounds -Werror=uninitialized -Werror=init-self -Werror=narrowing -Werror=return-type -Werror=switch -Werror=sign-compare -Werror=type-limits -Werror=undef -Werror=unused-result -Werror=unused-value -Werror=unused-variable -Werror=delete-non-virtual-dtor -Wfatal-errors -Wno-trigraphs -Werror=parentheses -DARDUPILOT_BUILD -Wuninitialized -Warray-bounds -Wno-format-contains-nul -Werror=unused-but-set-variable -fsingle-precision-constant -Wno-psabi -Werror=suggest-override -Werror=implicit-fallthrough -Werror=maybe-uninitialized -Werror=duplicated-cond -Werror=sizeof-pointer-div -Werror=use-after-free -D__AP_LINE__=__LINE__ -DAP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED=HAL_GCS_ENABLED -DAC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT=HAL_GCS_ENABLED -DHAL_NAVEKF2_AVAILABLE=1 -DCANARD_64_BIT=1 -Werror=float-equal -Werror=missing-declarations -DAP_NETWORKING_ENABLED=1 -O3 -MMD -ffile-prefix-map=../..=../.. -include ap_config.h -DHAL_HAVE_AP_ROMFS_EMBEDDED_H")

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

set(objs_AC_AttitudeControl_SOURCES
    libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.cpp
    libraries/AC_AttitudeControl/AC_AttitudeControl_TS.cpp
    libraries/AC_AttitudeControl/AC_CommandModel.cpp
    libraries/AC_AttitudeControl/AC_PosControl_Logging.cpp)

set(objs_AC_AttitudeControl_INCLUDES
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

add_library(objs_AC_AttitudeControl ${objs_AC_AttitudeControl_SOURCES}) #2
target_include_directories(objs_AC_AttitudeControl PRIVATE ${objs_AC_AttitudeControl_INCLUDES})

set_target_properties(objs_AC_AttitudeControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl littlefs) #clean

#------------------------------------------------

set(objs_AC_AutoTune_SOURCES
    libraries/AC_AutoTune/AC_AutoTune_FreqResp.cpp
    libraries/AC_AutoTune/AC_AutoTune_Heli.cpp)

set(objs_AC_AutoTune_INCLUDES
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

add_library(objs_AC_AutoTune ${objs_AC_AutoTune_SOURCES}) #2
target_include_directories(objs_AC_AutoTune PRIVATE ${objs_AC_AutoTune_INCLUDES})

set_target_properties(objs_AC_AutoTune PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AutoTune mavlink) #clean
target_link_libraries(objs_AC_AutoTune dronecan) #clean
target_link_libraries(objs_AC_AutoTune littlefs) #clean

#------------------------------------------------

set(objs_AC_Autorotation_SOURCES
    libraries/AC_Autorotation/AC_Autorotation.cpp
    libraries/AC_Autorotation/RSC_Autorotation.cpp)

set(objs_AC_Autorotation_INCLUDES
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

add_library(objs_AC_Autorotation ${objs_AC_Autorotation_SOURCES}) #2
target_include_directories(objs_AC_Autorotation PRIVATE ${objs_AC_Autorotation_INCLUDES})

set_target_properties(objs_AC_Autorotation PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Autorotation mavlink) #clean
target_link_libraries(objs_AC_Autorotation dronecan) #clean
target_link_libraries(objs_AC_Autorotation littlefs) #clean

#------------------------------------------------

set(objs_AC_Avoidance_SOURCES
    libraries/AC_Avoidance/AC_Avoidance_Logging.cpp
    libraries/AC_Avoidance/AP_OADijkstra.cpp
    libraries/AC_Avoidance/AP_OAPathPlanner.cpp
    libraries/AC_Avoidance/AP_OAVisGraph.cpp)

set(objs_AC_Avoidance_INCLUDES
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

add_library(objs_AC_Avoidance ${objs_AC_Avoidance_SOURCES}) #2
target_include_directories(objs_AC_Avoidance PRIVATE ${objs_AC_Avoidance_INCLUDES})

set_target_properties(objs_AC_Avoidance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance mavlink) #clean
target_link_libraries(objs_AC_Avoidance dronecan) #clean
target_link_libraries(objs_AC_Avoidance littlefs) #clean

#------------------------------------------------

set(objs_AC_CustomControl_SOURCES
    libraries/AC_CustomControl/AC_CustomControl.cpp
    libraries/AC_CustomControl/AC_CustomControl_Empty.cpp
    libraries/AC_CustomControl/AC_CustomControl_PID.cpp)

set(objs_AC_CustomControl_INCLUDES
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

add_library(objs_AC_CustomControl ${objs_AC_CustomControl_SOURCES}) #2
target_include_directories(objs_AC_CustomControl PRIVATE ${objs_AC_CustomControl_INCLUDES})

set_target_properties(objs_AC_CustomControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_CustomControl mavlink) #clean
target_link_libraries(objs_AC_CustomControl dronecan) #clean
target_link_libraries(objs_AC_CustomControl littlefs) #clean

#------------------------------------------------

set(objs_AC_InputManager_SOURCES
    libraries/AC_InputManager/AC_InputManager.cpp
    libraries/AC_InputManager/AC_InputManager_Heli.cpp)

set(objs_AC_InputManager_INCLUDES
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

add_library(objs_AC_InputManager ${objs_AC_InputManager_SOURCES}) #2
target_include_directories(objs_AC_InputManager PRIVATE ${objs_AC_InputManager_INCLUDES})

set_target_properties(objs_AC_InputManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_InputManager mavlink) #clean
target_link_libraries(objs_AC_InputManager dronecan) #clean
target_link_libraries(objs_AC_InputManager littlefs) #clean

#------------------------------------------------

set(objs_AC_PID_SOURCES
    libraries/AC_PID/AC_HELI_PID.cpp
    libraries/AC_PID/AC_P.cpp
    libraries/AC_PID/AC_PI.cpp
    libraries/AC_PID/AC_PID.cpp
    libraries/AC_PID/AC_PID_2D.cpp
    libraries/AC_PID/AC_PID_Basic.cpp
    libraries/AC_PID/AC_PI_2D.cpp
    libraries/AC_PID/AC_P_1D.cpp
    libraries/AC_PID/AC_P_2D.cpp)

set(objs_AC_PID_INCLUDES
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

add_library(objs_AC_PID ${objs_AC_PID_SOURCES}) #2
target_include_directories(objs_AC_PID PRIVATE ${objs_AC_PID_INCLUDES})

set_target_properties(objs_AC_PID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_PID mavlink) #clean
target_link_libraries(objs_AC_PID dronecan) #clean
target_link_libraries(objs_AC_PID littlefs) #clean

#------------------------------------------------

set(objs_AC_PrecLand_SOURCES
    libraries/AC_PrecLand/AC_PrecLand_IRLock.cpp
    libraries/AC_PrecLand/AC_PrecLand_MAVLink.cpp
    libraries/AC_PrecLand/AC_PrecLand_SITL.cpp
    libraries/AC_PrecLand/AC_PrecLand_SITL_Gazebo.cpp
    libraries/AC_PrecLand/AC_PrecLand_StateMachine.cpp
    libraries/AC_PrecLand/PosVelEKF.cpp)

set(objs_AC_PrecLand_INCLUDES
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

add_library(objs_AC_PrecLand ${objs_AC_PrecLand_SOURCES}) #2
target_include_directories(objs_AC_PrecLand PRIVATE ${objs_AC_PrecLand_INCLUDES})

set_target_properties(objs_AC_PrecLand PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_PrecLand mavlink) #clean
target_link_libraries(objs_AC_PrecLand dronecan) #clean
target_link_libraries(objs_AC_PrecLand littlefs) #clean

#------------------------------------------------

set(objs_AC_Sprayer_SOURCES
    libraries/AC_Sprayer/AC_Sprayer.cpp)

set(objs_AC_Sprayer_INCLUDES
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

add_library(objs_AC_Sprayer ${objs_AC_Sprayer_SOURCES}) #2
target_include_directories(objs_AC_Sprayer PRIVATE ${objs_AC_Sprayer_INCLUDES})

set_target_properties(objs_AC_Sprayer PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Sprayer mavlink) #clean
target_link_libraries(objs_AC_Sprayer dronecan) #clean
target_link_libraries(objs_AC_Sprayer littlefs) #clean

#------------------------------------------------

set(objs_AC_WPNav_SOURCES
    libraries/AC_WPNav/AC_Circle.cpp
    libraries/AC_WPNav/AC_WPNav.cpp
    libraries/AC_WPNav/AC_WPNav_OA.cpp)

set(objs_AC_WPNav_INCLUDES
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

add_library(objs_AC_WPNav ${objs_AC_WPNav_SOURCES}) #2
target_include_directories(objs_AC_WPNav PRIVATE ${objs_AC_WPNav_INCLUDES})

set_target_properties(objs_AC_WPNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav mavlink) #clean
target_link_libraries(objs_AC_WPNav dronecan) #clean
target_link_libraries(objs_AC_WPNav littlefs) #clean

#------------------------------------------------

set(objs_APM_Control_SOURCES
    libraries/APM_Control/AP_AutoTune.cpp
    libraries/APM_Control/AP_FW_Controller.cpp
    libraries/APM_Control/AP_PitchController.cpp
    libraries/APM_Control/AP_RollController.cpp
    libraries/APM_Control/AP_SteerController.cpp
    libraries/APM_Control/AP_YawController.cpp
    libraries/APM_Control/AR_AttitudeControl.cpp
    libraries/APM_Control/AR_PosControl.cpp)

set(objs_APM_Control_INCLUDES
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

add_library(objs_APM_Control ${objs_APM_Control_SOURCES}) #2
target_include_directories(objs_APM_Control PRIVATE ${objs_APM_Control_INCLUDES})

set_target_properties(objs_APM_Control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_APM_Control mavlink) #clean
target_link_libraries(objs_APM_Control dronecan) #clean
target_link_libraries(objs_APM_Control littlefs) #clean

#------------------------------------------------

set(objs_AP_ADC_SOURCES
    libraries/AP_ADC/AP_ADC_ADS1115.cpp)

set(objs_AP_ADC_INCLUDES
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

add_library(objs_AP_ADC ${objs_AP_ADC_SOURCES}) #2
target_include_directories(objs_AP_ADC PRIVATE ${objs_AP_ADC_INCLUDES})

set_target_properties(objs_AP_ADC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADC mavlink) #clean
target_link_libraries(objs_AP_ADC dronecan) #clean
target_link_libraries(objs_AP_ADC littlefs) #clean

#------------------------------------------------

set(objs_AP_ADSB_SOURCES
    libraries/AP_ADSB/AP_ADSB_Backend.cpp
    libraries/AP_ADSB/AP_ADSB_Sagetech.cpp
    libraries/AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp
    libraries/AP_ADSB/AP_ADSB_uAvionix_UCP.cpp)

set(objs_AP_ADSB_INCLUDES
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

add_library(objs_AP_ADSB ${objs_AP_ADSB_SOURCES}) #2
target_include_directories(objs_AP_ADSB PRIVATE ${objs_AP_ADSB_INCLUDES})

set_target_properties(objs_AP_ADSB PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB mavlink) #clean
target_link_libraries(objs_AP_ADSB dronecan) #clean
target_link_libraries(objs_AP_ADSB littlefs) #clean

#------------------------------------------------

set(objs_AP_AHRS_SOURCES
    libraries/AP_AHRS/AP_AHRS_External.cpp
    libraries/AP_AHRS/AP_AHRS_Logging.cpp
    libraries/AP_AHRS/AP_AHRS_SIM.cpp
    libraries/AP_AHRS/AP_AHRS_View.cpp)

set(objs_AP_AHRS_INCLUDES
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

add_library(objs_AP_AHRS ${objs_AP_AHRS_SOURCES}) #2
target_include_directories(objs_AP_AHRS PRIVATE ${objs_AP_AHRS_INCLUDES})

set_target_properties(objs_AP_AHRS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS mavlink) #clean
target_link_libraries(objs_AP_AHRS dronecan) #clean
target_link_libraries(objs_AP_AHRS littlefs) #clean

#------------------------------------------------

set(objs_AP_AccelCal_SOURCES
    libraries/AP_AccelCal/AP_AccelCal.cpp
    libraries/AP_AccelCal/AccelCalibrator.cpp)

set(objs_AP_AccelCal_INCLUDES
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

add_library(objs_AP_AccelCal ${objs_AP_AccelCal_SOURCES}) #2
target_include_directories(objs_AP_AccelCal PRIVATE ${objs_AP_AccelCal_INCLUDES})

set_target_properties(objs_AP_AccelCal PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AccelCal mavlink) #clean
target_link_libraries(objs_AP_AccelCal dronecan) #clean
target_link_libraries(objs_AP_AccelCal littlefs) #clean

#------------------------------------------------

set(objs_AP_AdvancedFailsafe_SOURCES
    libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp)

set(objs_AP_AdvancedFailsafe_INCLUDES
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

add_library(objs_AP_AdvancedFailsafe ${objs_AP_AdvancedFailsafe_SOURCES}) #2
target_include_directories(objs_AP_AdvancedFailsafe PRIVATE ${objs_AP_AdvancedFailsafe_INCLUDES})

set_target_properties(objs_AP_AdvancedFailsafe PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AdvancedFailsafe mavlink) #clean
target_link_libraries(objs_AP_AdvancedFailsafe dronecan) #clean
target_link_libraries(objs_AP_AdvancedFailsafe littlefs) #clean

#------------------------------------------------

set(objs_AP_Airspeed_SOURCES
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

set(objs_AP_Airspeed_INCLUDES
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

add_library(objs_AP_Airspeed ${objs_AP_Airspeed_SOURCES}) #2
target_include_directories(objs_AP_Airspeed PRIVATE ${objs_AP_Airspeed_INCLUDES})

set_target_properties(objs_AP_Airspeed PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed mavlink) #clean
target_link_libraries(objs_AP_Airspeed dronecan) #clean
target_link_libraries(objs_AP_Airspeed littlefs) #clean

#------------------------------------------------

set(objs_AP_Baro_SOURCES
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

set(objs_AP_Baro_INCLUDES
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

add_library(objs_AP_Baro ${objs_AP_Baro_SOURCES}) #2
target_include_directories(objs_AP_Baro PRIVATE ${objs_AP_Baro_INCLUDES})

set_target_properties(objs_AP_Baro PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro mavlink) #clean
target_link_libraries(objs_AP_Baro dronecan) #clean
target_link_libraries(objs_AP_Baro littlefs) #clean

#------------------------------------------------

set(objs_AP_BattMonitor_SOURCES
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

set(objs_AP_BattMonitor_INCLUDES
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

add_library(objs_AP_BattMonitor ${objs_AP_BattMonitor_SOURCES}) #2
target_include_directories(objs_AP_BattMonitor PRIVATE ${objs_AP_BattMonitor_INCLUDES})

set_target_properties(objs_AP_BattMonitor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor mavlink) #clean
target_link_libraries(objs_AP_BattMonitor dronecan) #clean
target_link_libraries(objs_AP_BattMonitor littlefs) #clean

#------------------------------------------------

set(objs_AP_Beacon_SOURCES
    libraries/AP_Beacon/AP_Beacon.cpp
    libraries/AP_Beacon/AP_Beacon_Backend.cpp
    libraries/AP_Beacon/AP_Beacon_Marvelmind.cpp
    libraries/AP_Beacon/AP_Beacon_Nooploop.cpp
    libraries/AP_Beacon/AP_Beacon_Pozyx.cpp
    libraries/AP_Beacon/AP_Beacon_SITL.cpp)

set(objs_AP_Beacon_INCLUDES
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

add_library(objs_AP_Beacon ${objs_AP_Beacon_SOURCES}) #2
target_include_directories(objs_AP_Beacon PRIVATE ${objs_AP_Beacon_INCLUDES})

set_target_properties(objs_AP_Beacon PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Beacon mavlink) #clean
target_link_libraries(objs_AP_Beacon dronecan) #clean
target_link_libraries(objs_AP_Beacon littlefs) #clean

#------------------------------------------------

set(objs_AP_BoardConfig_SOURCES
    libraries/AP_BoardConfig/IMU_heater.cpp
    libraries/AP_BoardConfig/board_drivers.cpp)

set(objs_AP_BoardConfig_INCLUDES
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

add_library(objs_AP_BoardConfig ${objs_AP_BoardConfig_SOURCES}) #2
target_include_directories(objs_AP_BoardConfig PRIVATE ${objs_AP_BoardConfig_INCLUDES})

set_target_properties(objs_AP_BoardConfig PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig mavlink) #clean
target_link_libraries(objs_AP_BoardConfig dronecan) #clean
target_link_libraries(objs_AP_BoardConfig littlefs) #clean

#------------------------------------------------

set(objs_AP_Button_SOURCES
    libraries/AP_Button/AP_Button.cpp)

set(objs_AP_Button_INCLUDES
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

add_library(objs_AP_Button ${objs_AP_Button_SOURCES}) #2
target_include_directories(objs_AP_Button PRIVATE ${objs_AP_Button_INCLUDES})

set_target_properties(objs_AP_Button PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Button mavlink) #clean
target_link_libraries(objs_AP_Button dronecan) #clean
target_link_libraries(objs_AP_Button littlefs) #clean

#------------------------------------------------

set(objs_AP_CANManager_SOURCES
    libraries/AP_CANManager/AP_CANIfaceParams.cpp
    libraries/AP_CANManager/AP_CANManager.cpp
    libraries/AP_CANManager/AP_CANManager_CANDriver_Params.cpp
    libraries/AP_CANManager/AP_CANSensor.cpp
    libraries/AP_CANManager/AP_MAVLinkCAN.cpp
    libraries/AP_CANManager/AP_SLCANIface.cpp)

set(objs_AP_CANManager_INCLUDES
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

add_library(objs_AP_CANManager ${objs_AP_CANManager_SOURCES}) #2
target_include_directories(objs_AP_CANManager PRIVATE ${objs_AP_CANManager_INCLUDES})

set_target_properties(objs_AP_CANManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_CANManager mavlink) #clean
target_link_libraries(objs_AP_CANManager dronecan) #clean
target_link_libraries(objs_AP_CANManager littlefs) #clean

#------------------------------------------------

set(objs_AP_CSVReader_SOURCES
    libraries/AP_CSVReader/AP_CSVReader.cpp)

set(objs_AP_CSVReader_INCLUDES
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

add_library(objs_AP_CSVReader ${objs_AP_CSVReader_SOURCES}) #2
target_include_directories(objs_AP_CSVReader PRIVATE ${objs_AP_CSVReader_INCLUDES})

set_target_properties(objs_AP_CSVReader PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_CSVReader mavlink) #clean
target_link_libraries(objs_AP_CSVReader dronecan) #clean
target_link_libraries(objs_AP_CSVReader littlefs) #clean

#------------------------------------------------

set(objs_AP_Camera_SOURCES
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

set(objs_AP_Camera_INCLUDES
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

add_library(objs_AP_Camera ${objs_AP_Camera_SOURCES}) #2
target_include_directories(objs_AP_Camera PRIVATE ${objs_AP_Camera_INCLUDES})

set_target_properties(objs_AP_Camera PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Camera mavlink) #clean
target_link_libraries(objs_AP_Camera dronecan) #clean
target_link_libraries(objs_AP_Camera littlefs) #clean

#------------------------------------------------

set(objs_AP_CheckFirmware_SOURCES
    libraries/AP_CheckFirmware/AP_CheckFirmware.cpp
    libraries/AP_CheckFirmware/AP_CheckFirmware_secure_command.cpp
    libraries/AP_CheckFirmware/monocypher.cpp)

set(objs_AP_CheckFirmware_INCLUDES
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

add_library(objs_AP_CheckFirmware ${objs_AP_CheckFirmware_SOURCES}) #2
target_include_directories(objs_AP_CheckFirmware PRIVATE ${objs_AP_CheckFirmware_INCLUDES})

set_target_properties(objs_AP_CheckFirmware PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_CheckFirmware mavlink) #clean
target_link_libraries(objs_AP_CheckFirmware dronecan) #clean
target_link_libraries(objs_AP_CheckFirmware littlefs) #clean

#------------------------------------------------

set(objs_AP_Common_SOURCES
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

set(objs_AP_Common_INCLUDES
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

add_library(objs_AP_Common ${objs_AP_Common_SOURCES}) #2
target_include_directories(objs_AP_Common PRIVATE ${objs_AP_Common_INCLUDES})

set_target_properties(objs_AP_Common PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Common mavlink) #clean
target_link_libraries(objs_AP_Common dronecan) #clean
target_link_libraries(objs_AP_Common littlefs) #clean

#------------------------------------------------

set(objs_AP_Compass_SOURCES
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

set(objs_AP_Compass_INCLUDES
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

add_library(objs_AP_Compass ${objs_AP_Compass_SOURCES}) #2
target_include_directories(objs_AP_Compass PRIVATE ${objs_AP_Compass_INCLUDES})

set_target_properties(objs_AP_Compass PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Compass mavlink) #clean
target_link_libraries(objs_AP_Compass dronecan) #clean
target_link_libraries(objs_AP_Compass littlefs) #clean

#------------------------------------------------

set(objs_AP_CustomRotations_SOURCES
    libraries/AP_CustomRotations/AP_CustomRotations.cpp
    libraries/AP_CustomRotations/AP_CustomRotations_params.cpp)

set(objs_AP_CustomRotations_INCLUDES
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

add_library(objs_AP_CustomRotations ${objs_AP_CustomRotations_SOURCES}) #2
target_include_directories(objs_AP_CustomRotations PRIVATE ${objs_AP_CustomRotations_INCLUDES})

set_target_properties(objs_AP_CustomRotations PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_CustomRotations mavlink) #clean
target_link_libraries(objs_AP_CustomRotations dronecan) #clean
target_link_libraries(objs_AP_CustomRotations littlefs) #clean

#------------------------------------------------

set(objs_AP_DAL_SOURCES
    libraries/AP_DAL/AP_DAL_Airspeed.cpp
    libraries/AP_DAL/AP_DAL_Baro.cpp
    libraries/AP_DAL/AP_DAL_Compass.cpp
    libraries/AP_DAL/AP_DAL_GPS.cpp
    libraries/AP_DAL/AP_DAL_InertialSensor.cpp)

set(objs_AP_DAL_INCLUDES
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

add_library(objs_AP_DAL ${objs_AP_DAL_SOURCES}) #2
target_include_directories(objs_AP_DAL PRIVATE ${objs_AP_DAL_INCLUDES})

set_target_properties(objs_AP_DAL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL mavlink) #clean
target_link_libraries(objs_AP_DAL dronecan) #clean
target_link_libraries(objs_AP_DAL littlefs) #clean

#------------------------------------------------

set(objs_AP_DDS_SOURCES
    libraries/AP_DDS/AP_DDS_Client.cpp
    libraries/AP_DDS/AP_DDS_ExternalControl.cpp
    libraries/AP_DDS/AP_DDS_External_Odom.cpp
    libraries/AP_DDS/AP_DDS_Serial.cpp
    libraries/AP_DDS/AP_DDS_Type_Conversions.cpp
    libraries/AP_DDS/AP_DDS_UDP.cpp)

set(objs_AP_DDS_INCLUDES
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

add_library(objs_AP_DDS ${objs_AP_DDS_SOURCES}) #2
target_include_directories(objs_AP_DDS PRIVATE ${objs_AP_DDS_INCLUDES})

set_target_properties(objs_AP_DDS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DDS mavlink) #clean
target_link_libraries(objs_AP_DDS dronecan) #clean
target_link_libraries(objs_AP_DDS littlefs) #clean

#------------------------------------------------

set(objs_AP_Declination_SOURCES
    libraries/AP_Declination/AP_Declination.cpp
    libraries/AP_Declination/tables.cpp)

set(objs_AP_Declination_INCLUDES
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

add_library(objs_AP_Declination ${objs_AP_Declination_SOURCES}) #2
target_include_directories(objs_AP_Declination PRIVATE ${objs_AP_Declination_INCLUDES})

set_target_properties(objs_AP_Declination PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Declination mavlink) #clean
target_link_libraries(objs_AP_Declination dronecan) #clean
target_link_libraries(objs_AP_Declination littlefs) #clean

#------------------------------------------------

set(objs_AP_Devo_Telem_SOURCES
    libraries/AP_Devo_Telem/AP_Devo_Telem.cpp)

set(objs_AP_Devo_Telem_INCLUDES
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

add_library(objs_AP_Devo_Telem ${objs_AP_Devo_Telem_SOURCES}) #2
target_include_directories(objs_AP_Devo_Telem PRIVATE ${objs_AP_Devo_Telem_INCLUDES})

set_target_properties(objs_AP_Devo_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Devo_Telem mavlink) #clean
target_link_libraries(objs_AP_Devo_Telem dronecan) #clean
target_link_libraries(objs_AP_Devo_Telem littlefs) #clean

#------------------------------------------------

set(objs_AP_DroneCAN_SOURCES
    libraries/AP_DroneCAN/AP_Canard_iface.cpp
    libraries/AP_DroneCAN/AP_DroneCAN.cpp
    libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp
    libraries/AP_DroneCAN/AP_DroneCAN_serial.cpp)

set(objs_AP_DroneCAN_INCLUDES
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

add_library(objs_AP_DroneCAN ${objs_AP_DroneCAN_SOURCES}) #2
target_include_directories(objs_AP_DroneCAN PRIVATE ${objs_AP_DroneCAN_INCLUDES})

set_target_properties(objs_AP_DroneCAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DroneCAN mavlink) #clean
target_link_libraries(objs_AP_DroneCAN dronecan) #clean
target_link_libraries(objs_AP_DroneCAN littlefs) #clean

#------------------------------------------------

set(objs_AP_EFI_SOURCES
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

set(objs_AP_EFI_INCLUDES
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

add_library(objs_AP_EFI ${objs_AP_EFI_SOURCES}) #2
target_include_directories(objs_AP_EFI PRIVATE ${objs_AP_EFI_INCLUDES})

set_target_properties(objs_AP_EFI PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_EFI mavlink) #clean
target_link_libraries(objs_AP_EFI dronecan) #clean
target_link_libraries(objs_AP_EFI littlefs) #clean

#------------------------------------------------

set(objs_AP_ESC_Telem_SOURCES
    libraries/AP_ESC_Telem/AP_ESC_Telem.cpp
    libraries/AP_ESC_Telem/AP_ESC_Telem_SITL.cpp)

set(objs_AP_ESC_Telem_INCLUDES
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

add_library(objs_AP_ESC_Telem ${objs_AP_ESC_Telem_SOURCES}) #2
target_include_directories(objs_AP_ESC_Telem PRIVATE ${objs_AP_ESC_Telem_INCLUDES})

set_target_properties(objs_AP_ESC_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem littlefs) #clean

#------------------------------------------------

set(objs_AP_ExternalAHRS_SOURCES
    libraries/AP_ExternalAHRS/AP_ExternalAHRS.cpp
    libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain5.cpp
    libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain7.cpp
    libraries/AP_ExternalAHRS/AP_ExternalAHRS_VectorNav.cpp
    libraries/AP_ExternalAHRS/AP_ExternalAHRS_backend.cpp
    libraries/AP_ExternalAHRS/MicroStrain_common.cpp)

set(objs_AP_ExternalAHRS_INCLUDES
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

add_library(objs_AP_ExternalAHRS ${objs_AP_ExternalAHRS_SOURCES}) #2
target_include_directories(objs_AP_ExternalAHRS PRIVATE ${objs_AP_ExternalAHRS_INCLUDES})

set_target_properties(objs_AP_ExternalAHRS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS littlefs) #clean

#------------------------------------------------

set(objs_AP_ExternalControl_SOURCES
    libraries/AP_ExternalControl/AP_ExternalControl.cpp)

set(objs_AP_ExternalControl_INCLUDES
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

add_library(objs_AP_ExternalControl ${objs_AP_ExternalControl_SOURCES}) #2
target_include_directories(objs_AP_ExternalControl PRIVATE ${objs_AP_ExternalControl_INCLUDES})

set_target_properties(objs_AP_ExternalControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalControl mavlink) #clean
target_link_libraries(objs_AP_ExternalControl dronecan) #clean
target_link_libraries(objs_AP_ExternalControl littlefs) #clean

#------------------------------------------------

set(objs_AP_FETtecOneWire_SOURCES
    libraries/AP_FETtecOneWire/AP_FETtecOneWire.cpp)

set(objs_AP_FETtecOneWire_INCLUDES
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

add_library(objs_AP_FETtecOneWire ${objs_AP_FETtecOneWire_SOURCES}) #2
target_include_directories(objs_AP_FETtecOneWire PRIVATE ${objs_AP_FETtecOneWire_INCLUDES})

set_target_properties(objs_AP_FETtecOneWire PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_FETtecOneWire mavlink) #clean
target_link_libraries(objs_AP_FETtecOneWire dronecan) #clean
target_link_libraries(objs_AP_FETtecOneWire littlefs) #clean

#------------------------------------------------

set(objs_AP_Filesystem_SOURCES
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

set(objs_AP_Filesystem_INCLUDES
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

add_library(objs_AP_Filesystem ${objs_AP_Filesystem_SOURCES}) #2
target_include_directories(objs_AP_Filesystem PRIVATE ${objs_AP_Filesystem_INCLUDES})

set_target_properties(objs_AP_Filesystem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem mavlink) #clean
target_link_libraries(objs_AP_Filesystem dronecan) #clean
target_link_libraries(objs_AP_Filesystem littlefs) #clean

#------------------------------------------------

set(objs_AP_FlashStorage_SOURCES
    libraries/AP_FlashStorage/AP_FlashStorage.cpp)

set(objs_AP_FlashStorage_INCLUDES
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

add_library(objs_AP_FlashStorage ${objs_AP_FlashStorage_SOURCES}) #2
target_include_directories(objs_AP_FlashStorage PRIVATE ${objs_AP_FlashStorage_INCLUDES})

set_target_properties(objs_AP_FlashStorage PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_FlashStorage mavlink) #clean
target_link_libraries(objs_AP_FlashStorage dronecan) #clean
target_link_libraries(objs_AP_FlashStorage littlefs) #clean

#------------------------------------------------

set(objs_AP_Frsky_Telem_SOURCES
    libraries/AP_Frsky_Telem/AP_Frsky_Backend.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_D.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_MAVliteToSPort.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_Message.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_SPortToMAVlite.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_Parameters.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_SPort.cpp
    libraries/AP_Frsky_Telem/AP_Frsky_Telem.cpp)

set(objs_AP_Frsky_Telem_INCLUDES
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

add_library(objs_AP_Frsky_Telem ${objs_AP_Frsky_Telem_SOURCES}) #2
target_include_directories(objs_AP_Frsky_Telem PRIVATE ${objs_AP_Frsky_Telem_INCLUDES})

set_target_properties(objs_AP_Frsky_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem littlefs) #clean

#------------------------------------------------

set(objs_AP_GPS_SOURCES
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

set(objs_AP_GPS_INCLUDES
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

add_library(objs_AP_GPS ${objs_AP_GPS_SOURCES}) #2
target_include_directories(objs_AP_GPS PRIVATE ${objs_AP_GPS_INCLUDES})

set_target_properties(objs_AP_GPS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GPS mavlink) #clean
target_link_libraries(objs_AP_GPS dronecan) #clean
target_link_libraries(objs_AP_GPS littlefs) #clean

#------------------------------------------------

set(objs_AP_GSOF_SOURCES
    libraries/AP_GSOF/AP_GSOF.cpp)

set(objs_AP_GSOF_INCLUDES
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

add_library(objs_AP_GSOF ${objs_AP_GSOF_SOURCES}) #2
target_include_directories(objs_AP_GSOF PRIVATE ${objs_AP_GSOF_INCLUDES})

set_target_properties(objs_AP_GSOF PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GSOF mavlink) #clean
target_link_libraries(objs_AP_GSOF dronecan) #clean
target_link_libraries(objs_AP_GSOF littlefs) #clean

#------------------------------------------------

set(objs_AP_Generator_SOURCES
    libraries/AP_Generator/AP_Generator.cpp
    libraries/AP_Generator/AP_Generator_Backend.cpp
    libraries/AP_Generator/AP_Generator_Cortex.cpp
    libraries/AP_Generator/AP_Generator_IE_2400.cpp
    libraries/AP_Generator/AP_Generator_IE_650_800.cpp
    libraries/AP_Generator/AP_Generator_IE_FuelCell.cpp
    libraries/AP_Generator/AP_Generator_Loweheiser.cpp
    libraries/AP_Generator/AP_Generator_RichenPower.cpp)

set(objs_AP_Generator_INCLUDES
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

add_library(objs_AP_Generator ${objs_AP_Generator_SOURCES}) #2
target_include_directories(objs_AP_Generator PRIVATE ${objs_AP_Generator_INCLUDES})

set_target_properties(objs_AP_Generator PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Generator mavlink) #clean
target_link_libraries(objs_AP_Generator dronecan) #clean
target_link_libraries(objs_AP_Generator littlefs) #clean

#------------------------------------------------

set(objs_AP_Gripper_SOURCES
    libraries/AP_Gripper/AP_Gripper.cpp
    libraries/AP_Gripper/AP_Gripper_Backend.cpp
    libraries/AP_Gripper/AP_Gripper_EPM.cpp
    libraries/AP_Gripper/AP_Gripper_Servo.cpp)

set(objs_AP_Gripper_INCLUDES
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

add_library(objs_AP_Gripper ${objs_AP_Gripper_SOURCES}) #2
target_include_directories(objs_AP_Gripper PRIVATE ${objs_AP_Gripper_INCLUDES})

set_target_properties(objs_AP_Gripper PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Gripper mavlink) #clean
target_link_libraries(objs_AP_Gripper dronecan) #clean
target_link_libraries(objs_AP_Gripper littlefs) #clean

#------------------------------------------------

set(objs_AP_Hott_Telem_SOURCES
    libraries/AP_Hott_Telem/AP_Hott_Telem.cpp)

set(objs_AP_Hott_Telem_INCLUDES
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

add_library(objs_AP_Hott_Telem ${objs_AP_Hott_Telem_SOURCES}) #2
target_include_directories(objs_AP_Hott_Telem PRIVATE ${objs_AP_Hott_Telem_INCLUDES})

set_target_properties(objs_AP_Hott_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Hott_Telem mavlink) #clean
target_link_libraries(objs_AP_Hott_Telem dronecan) #clean
target_link_libraries(objs_AP_Hott_Telem littlefs) #clean

#------------------------------------------------

set(objs_AP_ICEngine_SOURCES
    libraries/AP_ICEngine/AP_ICEngine.cpp
    libraries/AP_ICEngine/AP_ICEngine_TCA9554.cpp)

set(objs_AP_ICEngine_INCLUDES
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

add_library(objs_AP_ICEngine ${objs_AP_ICEngine_SOURCES}) #2
target_include_directories(objs_AP_ICEngine PRIVATE ${objs_AP_ICEngine_INCLUDES})

set_target_properties(objs_AP_ICEngine PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ICEngine mavlink) #clean
target_link_libraries(objs_AP_ICEngine dronecan) #clean
target_link_libraries(objs_AP_ICEngine littlefs) #clean

#------------------------------------------------

set(objs_AP_IOMCU_SOURCES
    libraries/AP_IOMCU/AP_IOMCU.cpp
    libraries/AP_IOMCU/fw_uploader.cpp)

set(objs_AP_IOMCU_INCLUDES
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

add_library(objs_AP_IOMCU ${objs_AP_IOMCU_SOURCES}) #2
target_include_directories(objs_AP_IOMCU PRIVATE ${objs_AP_IOMCU_INCLUDES})

set_target_properties(objs_AP_IOMCU PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IOMCU mavlink) #clean
target_link_libraries(objs_AP_IOMCU dronecan) #clean
target_link_libraries(objs_AP_IOMCU littlefs) #clean

#------------------------------------------------

set(objs_AP_IRLock_SOURCES
    libraries/AP_IRLock/AP_IRLock.cpp
    libraries/AP_IRLock/AP_IRLock_I2C.cpp
    libraries/AP_IRLock/AP_IRLock_SITL.cpp
    libraries/AP_IRLock/AP_IRLock_SITL_Gazebo.cpp)

set(objs_AP_IRLock_INCLUDES
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

add_library(objs_AP_IRLock ${objs_AP_IRLock_SOURCES}) #2
target_include_directories(objs_AP_IRLock PRIVATE ${objs_AP_IRLock_INCLUDES})

set_target_properties(objs_AP_IRLock PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IRLock mavlink) #clean
target_link_libraries(objs_AP_IRLock dronecan) #clean
target_link_libraries(objs_AP_IRLock littlefs) #clean

#------------------------------------------------

set(objs_AP_InertialNav_SOURCES
    libraries/AP_InertialNav/AP_InertialNav.cpp)

set(objs_AP_InertialNav_INCLUDES
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

add_library(objs_AP_InertialNav ${objs_AP_InertialNav_SOURCES}) #2
target_include_directories(objs_AP_InertialNav PRIVATE ${objs_AP_InertialNav_INCLUDES})

set_target_properties(objs_AP_InertialNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialNav mavlink) #clean
target_link_libraries(objs_AP_InertialNav dronecan) #clean
target_link_libraries(objs_AP_InertialNav littlefs) #clean

#------------------------------------------------

set(objs_AP_InertialSensor_SOURCES
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

set(objs_AP_InertialSensor_INCLUDES
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

add_library(objs_AP_InertialSensor ${objs_AP_InertialSensor_SOURCES}) #2
target_include_directories(objs_AP_InertialSensor PRIVATE ${objs_AP_InertialSensor_INCLUDES})

set_target_properties(objs_AP_InertialSensor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor mavlink) #clean
target_link_libraries(objs_AP_InertialSensor dronecan) #clean
target_link_libraries(objs_AP_InertialSensor littlefs) #clean

#------------------------------------------------

set(objs_AP_InternalError_SOURCES
    libraries/AP_InternalError/AP_InternalError.cpp)

set(objs_AP_InternalError_INCLUDES
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

add_library(objs_AP_InternalError ${objs_AP_InternalError_SOURCES}) #2
target_include_directories(objs_AP_InternalError PRIVATE ${objs_AP_InternalError_INCLUDES})

set_target_properties(objs_AP_InternalError PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InternalError mavlink) #clean
target_link_libraries(objs_AP_InternalError dronecan) #clean
target_link_libraries(objs_AP_InternalError littlefs) #clean

#------------------------------------------------

set(objs_AP_JSButton_SOURCES
    libraries/AP_JSButton/AP_JSButton.cpp)

set(objs_AP_JSButton_INCLUDES
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

add_library(objs_AP_JSButton ${objs_AP_JSButton_SOURCES}) #2
target_include_directories(objs_AP_JSButton PRIVATE ${objs_AP_JSButton_INCLUDES})

set_target_properties(objs_AP_JSButton PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_JSButton mavlink) #clean
target_link_libraries(objs_AP_JSButton dronecan) #clean
target_link_libraries(objs_AP_JSButton littlefs) #clean

#------------------------------------------------

set(objs_AP_JSON_SOURCES
    libraries/AP_JSON/AP_JSON.cpp)

set(objs_AP_JSON_INCLUDES
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

add_library(objs_AP_JSON ${objs_AP_JSON_SOURCES}) #2
target_include_directories(objs_AP_JSON PRIVATE ${objs_AP_JSON_INCLUDES})

set_target_properties(objs_AP_JSON PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_JSON mavlink) #clean
target_link_libraries(objs_AP_JSON dronecan) #clean
target_link_libraries(objs_AP_JSON littlefs) #clean

#------------------------------------------------

set(objs_AP_KDECAN_SOURCES
    libraries/AP_KDECAN/AP_KDECAN.cpp)

set(objs_AP_KDECAN_INCLUDES
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

add_library(objs_AP_KDECAN ${objs_AP_KDECAN_SOURCES}) #2
target_include_directories(objs_AP_KDECAN PRIVATE ${objs_AP_KDECAN_INCLUDES})

set_target_properties(objs_AP_KDECAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_KDECAN mavlink) #clean
target_link_libraries(objs_AP_KDECAN dronecan) #clean
target_link_libraries(objs_AP_KDECAN littlefs) #clean

#------------------------------------------------

set(objs_AP_L1_Control_SOURCES
    libraries/AP_L1_Control/AP_L1_Control.cpp)

set(objs_AP_L1_Control_INCLUDES
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

add_library(objs_AP_L1_Control ${objs_AP_L1_Control_SOURCES}) #2
target_include_directories(objs_AP_L1_Control PRIVATE ${objs_AP_L1_Control_INCLUDES})

set_target_properties(objs_AP_L1_Control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_L1_Control mavlink) #clean
target_link_libraries(objs_AP_L1_Control dronecan) #clean
target_link_libraries(objs_AP_L1_Control littlefs) #clean

#------------------------------------------------

set(objs_AP_LTM_Telem_SOURCES
    libraries/AP_LTM_Telem/AP_LTM_Telem.cpp)

set(objs_AP_LTM_Telem_INCLUDES
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

add_library(objs_AP_LTM_Telem ${objs_AP_LTM_Telem_SOURCES}) #2
target_include_directories(objs_AP_LTM_Telem PRIVATE ${objs_AP_LTM_Telem_INCLUDES})

set_target_properties(objs_AP_LTM_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LTM_Telem mavlink) #clean
target_link_libraries(objs_AP_LTM_Telem dronecan) #clean
target_link_libraries(objs_AP_LTM_Telem littlefs) #clean

#------------------------------------------------

set(objs_AP_Landing_SOURCES
    libraries/AP_Landing/AP_Landing.cpp
    libraries/AP_Landing/AP_Landing_Deepstall.cpp)

set(objs_AP_Landing_INCLUDES
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

add_library(objs_AP_Landing ${objs_AP_Landing_SOURCES}) #2
target_include_directories(objs_AP_Landing PRIVATE ${objs_AP_Landing_INCLUDES})

set_target_properties(objs_AP_Landing PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Landing mavlink) #clean
target_link_libraries(objs_AP_Landing dronecan) #clean
target_link_libraries(objs_AP_Landing littlefs) #clean

#------------------------------------------------

set(objs_AP_LeakDetector_SOURCES
    libraries/AP_LeakDetector/AP_LeakDetector.cpp
    libraries/AP_LeakDetector/AP_LeakDetector_Analog.cpp
    libraries/AP_LeakDetector/AP_LeakDetector_Backend.cpp
    libraries/AP_LeakDetector/AP_LeakDetector_Digital.cpp)

set(objs_AP_LeakDetector_INCLUDES
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

add_library(objs_AP_LeakDetector ${objs_AP_LeakDetector_SOURCES}) #2
target_include_directories(objs_AP_LeakDetector PRIVATE ${objs_AP_LeakDetector_INCLUDES})

set_target_properties(objs_AP_LeakDetector PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LeakDetector mavlink) #clean
target_link_libraries(objs_AP_LeakDetector dronecan) #clean
target_link_libraries(objs_AP_LeakDetector littlefs) #clean

#------------------------------------------------

set(objs_AP_LightWareSerial_SOURCES
    libraries/AP_LightWareSerial/AP_LightWareSerial.cpp)

set(objs_AP_LightWareSerial_INCLUDES
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

add_library(objs_AP_LightWareSerial ${objs_AP_LightWareSerial_SOURCES}) #2
target_include_directories(objs_AP_LightWareSerial PRIVATE ${objs_AP_LightWareSerial_INCLUDES})

set_target_properties(objs_AP_LightWareSerial PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LightWareSerial mavlink) #clean
target_link_libraries(objs_AP_LightWareSerial dronecan) #clean
target_link_libraries(objs_AP_LightWareSerial littlefs) #clean

#------------------------------------------------

set(objs_AP_Logger_SOURCES
    libraries/AP_Logger/AP_Logger_Block.cpp
    libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp
    libraries/AP_Logger/AP_Logger_MAVLink.cpp
    libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp
    libraries/AP_Logger/AP_Logger_W25NXX.cpp
    libraries/AP_Logger/LogFile.cpp)

set(objs_AP_Logger_INCLUDES
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

add_library(objs_AP_Logger ${objs_AP_Logger_SOURCES}) #2
target_include_directories(objs_AP_Logger PRIVATE ${objs_AP_Logger_INCLUDES})

set_target_properties(objs_AP_Logger PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger mavlink) #clean
target_link_libraries(objs_AP_Logger dronecan) #clean
target_link_libraries(objs_AP_Logger littlefs) #clean

#------------------------------------------------

set(objs_AP_MSP_SOURCES
    libraries/AP_MSP/AP_MSP.cpp
    libraries/AP_MSP/AP_MSP_Telem_Backend.cpp
    libraries/AP_MSP/AP_MSP_Telem_DJI.cpp
    libraries/AP_MSP/AP_MSP_Telem_DisplayPort.cpp
    libraries/AP_MSP/AP_MSP_Telem_Generic.cpp
    libraries/AP_MSP/msp.cpp
    libraries/AP_MSP/msp_sbuf.cpp)

set(objs_AP_MSP_INCLUDES
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

add_library(objs_AP_MSP ${objs_AP_MSP_SOURCES}) #2
target_include_directories(objs_AP_MSP PRIVATE ${objs_AP_MSP_INCLUDES})

set_target_properties(objs_AP_MSP PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_MSP mavlink) #clean
target_link_libraries(objs_AP_MSP dronecan) #clean
target_link_libraries(objs_AP_MSP littlefs) #clean

#------------------------------------------------

set(objs_AP_Math_SOURCES
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

set(objs_AP_Math_INCLUDES
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

add_library(objs_AP_Math ${objs_AP_Math_SOURCES}) #2
target_include_directories(objs_AP_Math PRIVATE ${objs_AP_Math_INCLUDES})

set_target_properties(objs_AP_Math PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math mavlink) #clean
target_link_libraries(objs_AP_Math dronecan) #clean
target_link_libraries(objs_AP_Math littlefs) #clean

#------------------------------------------------

set(objs_AP_Mission_SOURCES
    libraries/AP_Mission/AP_Mission_ChangeDetector.cpp
    libraries/AP_Mission/AP_Mission_Commands.cpp)

set(objs_AP_Mission_INCLUDES
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

add_library(objs_AP_Mission ${objs_AP_Mission_SOURCES}) #2
target_include_directories(objs_AP_Mission PRIVATE ${objs_AP_Mission_INCLUDES})

set_target_properties(objs_AP_Mission PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission mavlink) #clean
target_link_libraries(objs_AP_Mission dronecan) #clean
target_link_libraries(objs_AP_Mission littlefs) #clean

#------------------------------------------------

set(objs_AP_Module_SOURCES
    libraries/AP_Module/AP_Module.cpp)

set(objs_AP_Module_INCLUDES
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

add_library(objs_AP_Module ${objs_AP_Module_SOURCES}) #2
target_include_directories(objs_AP_Module PRIVATE ${objs_AP_Module_INCLUDES})

set_target_properties(objs_AP_Module PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Module mavlink) #clean
target_link_libraries(objs_AP_Module dronecan) #clean
target_link_libraries(objs_AP_Module littlefs) #clean

#------------------------------------------------

set(objs_AP_Motors_SOURCES
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

set(objs_AP_Motors_INCLUDES
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

add_library(objs_AP_Motors ${objs_AP_Motors_SOURCES}) #2
target_include_directories(objs_AP_Motors PRIVATE ${objs_AP_Motors_INCLUDES})

set_target_properties(objs_AP_Motors PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors mavlink) #clean
target_link_libraries(objs_AP_Motors dronecan) #clean
target_link_libraries(objs_AP_Motors littlefs) #clean

#------------------------------------------------

set(objs_AP_Mount_SOURCES
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

set(objs_AP_Mount_INCLUDES
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

add_library(objs_AP_Mount ${objs_AP_Mount_SOURCES}) #2
target_include_directories(objs_AP_Mount PRIVATE ${objs_AP_Mount_INCLUDES})

set_target_properties(objs_AP_Mount PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mount mavlink) #clean
target_link_libraries(objs_AP_Mount dronecan) #clean
target_link_libraries(objs_AP_Mount littlefs) #clean

#------------------------------------------------

set(objs_AP_MultiHeap_SOURCES
    libraries/AP_MultiHeap/AP_MultiHeap.cpp
    libraries/AP_MultiHeap/MultiHeap_chibios.cpp
    libraries/AP_MultiHeap/MultiHeap_malloc.cpp)

set(objs_AP_MultiHeap_INCLUDES
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

add_library(objs_AP_MultiHeap ${objs_AP_MultiHeap_SOURCES}) #2
target_include_directories(objs_AP_MultiHeap PRIVATE ${objs_AP_MultiHeap_INCLUDES})

set_target_properties(objs_AP_MultiHeap PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_MultiHeap mavlink) #clean
target_link_libraries(objs_AP_MultiHeap dronecan) #clean
target_link_libraries(objs_AP_MultiHeap littlefs) #clean

#------------------------------------------------

set(objs_AP_NMEA_Output_SOURCES
    libraries/AP_NMEA_Output/AP_NMEA_Output.cpp)

set(objs_AP_NMEA_Output_INCLUDES
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

add_library(objs_AP_NMEA_Output ${objs_AP_NMEA_Output_SOURCES}) #2
target_include_directories(objs_AP_NMEA_Output PRIVATE ${objs_AP_NMEA_Output_INCLUDES})

set_target_properties(objs_AP_NMEA_Output PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NMEA_Output mavlink) #clean
target_link_libraries(objs_AP_NMEA_Output dronecan) #clean
target_link_libraries(objs_AP_NMEA_Output littlefs) #clean

#------------------------------------------------

set(objs_AP_NavEKF_SOURCES
    libraries/AP_NavEKF/AP_NavEKF_Source.cpp
    libraries/AP_NavEKF/AP_NavEKF_core_common.cpp
    libraries/AP_NavEKF/EKFGSF_Logging.cpp
    libraries/AP_NavEKF/EKFGSF_yaw.cpp
    libraries/AP_NavEKF/EKF_Buffer.cpp)

set(objs_AP_NavEKF_INCLUDES
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

add_library(objs_AP_NavEKF ${objs_AP_NavEKF_SOURCES}) #2
target_include_directories(objs_AP_NavEKF PRIVATE ${objs_AP_NavEKF_INCLUDES})

set_target_properties(objs_AP_NavEKF PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF mavlink) #clean
target_link_libraries(objs_AP_NavEKF dronecan) #clean
target_link_libraries(objs_AP_NavEKF littlefs) #clean

#------------------------------------------------

set(objs_AP_NavEKF2_SOURCES
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

set(objs_AP_NavEKF2_INCLUDES
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

add_library(objs_AP_NavEKF2 ${objs_AP_NavEKF2_SOURCES}) #2
target_include_directories(objs_AP_NavEKF2 PRIVATE ${objs_AP_NavEKF2_INCLUDES})

set_target_properties(objs_AP_NavEKF2 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2 mavlink) #clean
target_link_libraries(objs_AP_NavEKF2 dronecan) #clean
target_link_libraries(objs_AP_NavEKF2 littlefs) #clean

#------------------------------------------------

set(objs_AP_Networking_SOURCES
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

set(objs_AP_Networking_INCLUDES
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

add_library(objs_AP_Networking ${objs_AP_Networking_SOURCES}) #2
target_include_directories(objs_AP_Networking PRIVATE ${objs_AP_Networking_INCLUDES})

set_target_properties(objs_AP_Networking PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Networking mavlink) #clean
target_link_libraries(objs_AP_Networking dronecan) #clean
target_link_libraries(objs_AP_Networking littlefs) #clean

#------------------------------------------------

set(objs_AP_Notify_SOURCES
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

set(objs_AP_Notify_INCLUDES
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

add_library(objs_AP_Notify ${objs_AP_Notify_SOURCES}) #2
target_include_directories(objs_AP_Notify PRIVATE ${objs_AP_Notify_INCLUDES})

set_target_properties(objs_AP_Notify PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Notify mavlink) #clean
target_link_libraries(objs_AP_Notify dronecan) #clean
target_link_libraries(objs_AP_Notify littlefs) #clean

#------------------------------------------------

set(objs_AP_OLC_SOURCES
    libraries/AP_OLC/AP_OLC.cpp)

set(objs_AP_OLC_INCLUDES
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

add_library(objs_AP_OLC ${objs_AP_OLC_SOURCES}) #2
target_include_directories(objs_AP_OLC PRIVATE ${objs_AP_OLC_INCLUDES})

set_target_properties(objs_AP_OLC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OLC mavlink) #clean
target_link_libraries(objs_AP_OLC dronecan) #clean
target_link_libraries(objs_AP_OLC littlefs) #clean

#------------------------------------------------

set(objs_AP_OSD_SOURCES
    libraries/AP_OSD/AP_OSD.cpp
    libraries/AP_OSD/AP_OSD_Backend.cpp
    libraries/AP_OSD/AP_OSD_MAX7456.cpp
    libraries/AP_OSD/AP_OSD_MSP.cpp
    libraries/AP_OSD/AP_OSD_MSP_DisplayPort.cpp
    libraries/AP_OSD/AP_OSD_SITL.cpp
    libraries/AP_OSD/AP_OSD_Setting.cpp)

set(objs_AP_OSD_INCLUDES
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

add_library(objs_AP_OSD ${objs_AP_OSD_SOURCES}) #2
target_include_directories(objs_AP_OSD PRIVATE ${objs_AP_OSD_INCLUDES})

set_target_properties(objs_AP_OSD PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD mavlink) #clean
target_link_libraries(objs_AP_OSD dronecan) #clean
target_link_libraries(objs_AP_OSD littlefs) #clean

#------------------------------------------------

set(objs_AP_OpenDroneID_SOURCES
    libraries/AP_OpenDroneID/AP_OpenDroneID.cpp
    libraries/AP_OpenDroneID/AP_OpenDroneID_DroneCAN.cpp)

set(objs_AP_OpenDroneID_INCLUDES
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

add_library(objs_AP_OpenDroneID ${objs_AP_OpenDroneID_SOURCES}) #2
target_include_directories(objs_AP_OpenDroneID PRIVATE ${objs_AP_OpenDroneID_INCLUDES})

set_target_properties(objs_AP_OpenDroneID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OpenDroneID mavlink) #clean
target_link_libraries(objs_AP_OpenDroneID dronecan) #clean
target_link_libraries(objs_AP_OpenDroneID littlefs) #clean

#------------------------------------------------

set(objs_AP_OpticalFlow_SOURCES
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

set(objs_AP_OpticalFlow_INCLUDES
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

add_library(objs_AP_OpticalFlow ${objs_AP_OpticalFlow_SOURCES}) #2
target_include_directories(objs_AP_OpticalFlow PRIVATE ${objs_AP_OpticalFlow_INCLUDES})

set_target_properties(objs_AP_OpticalFlow PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OpticalFlow mavlink) #clean
target_link_libraries(objs_AP_OpticalFlow dronecan) #clean
target_link_libraries(objs_AP_OpticalFlow littlefs) #clean

#------------------------------------------------

set(objs_AP_Parachute_SOURCES
    libraries/AP_Parachute/AP_Parachute.cpp)

set(objs_AP_Parachute_INCLUDES
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

add_library(objs_AP_Parachute ${objs_AP_Parachute_SOURCES}) #2
target_include_directories(objs_AP_Parachute PRIVATE ${objs_AP_Parachute_INCLUDES})

set_target_properties(objs_AP_Parachute PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Parachute mavlink) #clean
target_link_libraries(objs_AP_Parachute dronecan) #clean
target_link_libraries(objs_AP_Parachute littlefs) #clean

#------------------------------------------------

set(objs_AP_Param_SOURCES
    libraries/AP_Param/AP_Param.cpp
    libraries/AP_Param/AP_ParamT.cpp)

set(objs_AP_Param_INCLUDES
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

add_library(objs_AP_Param ${objs_AP_Param_SOURCES}) #2
target_include_directories(objs_AP_Param PRIVATE ${objs_AP_Param_INCLUDES})

set_target_properties(objs_AP_Param PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Param mavlink) #clean
target_link_libraries(objs_AP_Param dronecan) #clean
target_link_libraries(objs_AP_Param littlefs) #clean

#------------------------------------------------

set(objs_AP_PiccoloCAN_SOURCES
    libraries/AP_PiccoloCAN/AP_PiccoloCAN.cpp
    libraries/AP_PiccoloCAN/AP_PiccoloCAN_Cortex.cpp
    libraries/AP_PiccoloCAN/AP_PiccoloCAN_ECU.cpp
    libraries/AP_PiccoloCAN/AP_PiccoloCAN_ESC.cpp
    libraries/AP_PiccoloCAN/AP_PiccoloCAN_Servo.cpp)

set(objs_AP_PiccoloCAN_INCLUDES
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

add_library(objs_AP_PiccoloCAN ${objs_AP_PiccoloCAN_SOURCES}) #2
target_include_directories(objs_AP_PiccoloCAN PRIVATE ${objs_AP_PiccoloCAN_INCLUDES})

set_target_properties(objs_AP_PiccoloCAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_PiccoloCAN mavlink) #clean
target_link_libraries(objs_AP_PiccoloCAN dronecan) #clean
target_link_libraries(objs_AP_PiccoloCAN littlefs) #clean

#------------------------------------------------

set(objs_AP_Proximity_SOURCES
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

set(objs_AP_Proximity_INCLUDES
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

add_library(objs_AP_Proximity ${objs_AP_Proximity_SOURCES}) #2
target_include_directories(objs_AP_Proximity PRIVATE ${objs_AP_Proximity_INCLUDES})

set_target_properties(objs_AP_Proximity PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity mavlink) #clean
target_link_libraries(objs_AP_Proximity dronecan) #clean
target_link_libraries(objs_AP_Proximity littlefs) #clean

#------------------------------------------------

set(objs_AP_Quicktune_SOURCES
    libraries/AP_Quicktune/AP_Quicktune.cpp)

set(objs_AP_Quicktune_INCLUDES
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

add_library(objs_AP_Quicktune ${objs_AP_Quicktune_SOURCES}) #2
target_include_directories(objs_AP_Quicktune PRIVATE ${objs_AP_Quicktune_INCLUDES})

set_target_properties(objs_AP_Quicktune PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Quicktune mavlink) #clean
target_link_libraries(objs_AP_Quicktune dronecan) #clean
target_link_libraries(objs_AP_Quicktune littlefs) #clean

#------------------------------------------------

set(objs_AP_RAMTRON_SOURCES
    libraries/AP_RAMTRON/AP_RAMTRON.cpp)

set(objs_AP_RAMTRON_INCLUDES
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

add_library(objs_AP_RAMTRON ${objs_AP_RAMTRON_SOURCES}) #2
target_include_directories(objs_AP_RAMTRON PRIVATE ${objs_AP_RAMTRON_INCLUDES})

set_target_properties(objs_AP_RAMTRON PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RAMTRON mavlink) #clean
target_link_libraries(objs_AP_RAMTRON dronecan) #clean
target_link_libraries(objs_AP_RAMTRON littlefs) #clean

#------------------------------------------------

set(objs_AP_RCMapper_SOURCES
    libraries/AP_RCMapper/AP_RCMapper.cpp)

set(objs_AP_RCMapper_INCLUDES
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

add_library(objs_AP_RCMapper ${objs_AP_RCMapper_SOURCES}) #2
target_include_directories(objs_AP_RCMapper PRIVATE ${objs_AP_RCMapper_INCLUDES})

set_target_properties(objs_AP_RCMapper PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCMapper mavlink) #clean
target_link_libraries(objs_AP_RCMapper dronecan) #clean
target_link_libraries(objs_AP_RCMapper littlefs) #clean

#------------------------------------------------

set(objs_AP_RCProtocol_SOURCES
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

set(objs_AP_RCProtocol_INCLUDES
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

add_library(objs_AP_RCProtocol ${objs_AP_RCProtocol_SOURCES}) #2
target_include_directories(objs_AP_RCProtocol PRIVATE ${objs_AP_RCProtocol_INCLUDES})

set_target_properties(objs_AP_RCProtocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol mavlink) #clean
target_link_libraries(objs_AP_RCProtocol dronecan) #clean
target_link_libraries(objs_AP_RCProtocol littlefs) #clean

#------------------------------------------------

set(objs_AP_RCTelemetry_SOURCES
    libraries/AP_RCTelemetry/AP_GHST_Telem.cpp
    libraries/AP_RCTelemetry/AP_Spektrum_Telem.cpp)

set(objs_AP_RCTelemetry_INCLUDES
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

add_library(objs_AP_RCTelemetry ${objs_AP_RCTelemetry_SOURCES}) #2
target_include_directories(objs_AP_RCTelemetry PRIVATE ${objs_AP_RCTelemetry_INCLUDES})

set_target_properties(objs_AP_RCTelemetry PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry littlefs) #clean

#------------------------------------------------

set(objs_AP_ROMFS_SOURCES
    libraries/AP_ROMFS/AP_ROMFS.cpp
    libraries/AP_ROMFS/tinflate.cpp)

set(objs_AP_ROMFS_INCLUDES
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

add_library(objs_AP_ROMFS ${objs_AP_ROMFS_SOURCES}) #2
target_include_directories(objs_AP_ROMFS PRIVATE ${objs_AP_ROMFS_INCLUDES})

set_target_properties(objs_AP_ROMFS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ROMFS mavlink) #clean
target_link_libraries(objs_AP_ROMFS dronecan) #clean
target_link_libraries(objs_AP_ROMFS littlefs) #clean

#------------------------------------------------

set(objs_AP_RPM_SOURCES
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

set(objs_AP_RPM_INCLUDES
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

add_library(objs_AP_RPM ${objs_AP_RPM_SOURCES}) #2
target_include_directories(objs_AP_RPM PRIVATE ${objs_AP_RPM_INCLUDES})

set_target_properties(objs_AP_RPM PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RPM mavlink) #clean
target_link_libraries(objs_AP_RPM dronecan) #clean
target_link_libraries(objs_AP_RPM littlefs) #clean

#------------------------------------------------

set(objs_AP_RSSI_SOURCES
    libraries/AP_RSSI/AP_RSSI.cpp)

set(objs_AP_RSSI_INCLUDES
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

add_library(objs_AP_RSSI ${objs_AP_RSSI_SOURCES}) #2
target_include_directories(objs_AP_RSSI PRIVATE ${objs_AP_RSSI_INCLUDES})

set_target_properties(objs_AP_RSSI PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RSSI mavlink) #clean
target_link_libraries(objs_AP_RSSI dronecan) #clean
target_link_libraries(objs_AP_RSSI littlefs) #clean

#------------------------------------------------

set(objs_AP_RTC_SOURCES
    libraries/AP_RTC/AP_RTC.cpp
    libraries/AP_RTC/JitterCorrection.cpp)

set(objs_AP_RTC_INCLUDES
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

add_library(objs_AP_RTC ${objs_AP_RTC_SOURCES}) #2
target_include_directories(objs_AP_RTC PRIVATE ${objs_AP_RTC_INCLUDES})

set_target_properties(objs_AP_RTC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RTC mavlink) #clean
target_link_libraries(objs_AP_RTC dronecan) #clean
target_link_libraries(objs_AP_RTC littlefs) #clean

#------------------------------------------------

set(objs_AP_Radio_SOURCES
    libraries/AP_Radio/AP_Radio.cpp
    libraries/AP_Radio/AP_Radio_backend.cpp
    libraries/AP_Radio/AP_Radio_bk2425.cpp
    libraries/AP_Radio/AP_Radio_cc2500.cpp
    libraries/AP_Radio/AP_Radio_cypress.cpp
    libraries/AP_Radio/driver_bk2425.cpp
    libraries/AP_Radio/driver_cc2500.cpp)

set(objs_AP_Radio_INCLUDES
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

add_library(objs_AP_Radio ${objs_AP_Radio_SOURCES}) #2
target_include_directories(objs_AP_Radio PRIVATE ${objs_AP_Radio_INCLUDES})

set_target_properties(objs_AP_Radio PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Radio mavlink) #clean
target_link_libraries(objs_AP_Radio dronecan) #clean
target_link_libraries(objs_AP_Radio littlefs) #clean

#------------------------------------------------

set(objs_AP_RangeFinder_SOURCES
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

set(objs_AP_RangeFinder_INCLUDES
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

add_library(objs_AP_RangeFinder ${objs_AP_RangeFinder_SOURCES}) #2
target_include_directories(objs_AP_RangeFinder PRIVATE ${objs_AP_RangeFinder_INCLUDES})

set_target_properties(objs_AP_RangeFinder PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RangeFinder mavlink) #clean
target_link_libraries(objs_AP_RangeFinder dronecan) #clean
target_link_libraries(objs_AP_RangeFinder littlefs) #clean

#------------------------------------------------

set(objs_AP_Relay_SOURCES
    libraries/AP_Relay/AP_Relay_Params.cpp)

set(objs_AP_Relay_INCLUDES
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

add_library(objs_AP_Relay ${objs_AP_Relay_SOURCES}) #2
target_include_directories(objs_AP_Relay PRIVATE ${objs_AP_Relay_INCLUDES})

set_target_properties(objs_AP_Relay PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay mavlink) #clean
target_link_libraries(objs_AP_Relay dronecan) #clean
target_link_libraries(objs_AP_Relay littlefs) #clean

#------------------------------------------------

set(objs_AP_RobotisServo_SOURCES
    libraries/AP_RobotisServo/AP_RobotisServo.cpp)

set(objs_AP_RobotisServo_INCLUDES
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

add_library(objs_AP_RobotisServo ${objs_AP_RobotisServo_SOURCES}) #2
target_include_directories(objs_AP_RobotisServo PRIVATE ${objs_AP_RobotisServo_INCLUDES})

set_target_properties(objs_AP_RobotisServo PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RobotisServo mavlink) #clean
target_link_libraries(objs_AP_RobotisServo dronecan) #clean
target_link_libraries(objs_AP_RobotisServo littlefs) #clean

#------------------------------------------------

set(objs_AP_SBusOut_SOURCES
    libraries/AP_SBusOut/AP_SBusOut.cpp)

set(objs_AP_SBusOut_INCLUDES
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

add_library(objs_AP_SBusOut ${objs_AP_SBusOut_SOURCES}) #2
target_include_directories(objs_AP_SBusOut PRIVATE ${objs_AP_SBusOut_INCLUDES})

set_target_properties(objs_AP_SBusOut PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_SBusOut mavlink) #clean
target_link_libraries(objs_AP_SBusOut dronecan) #clean
target_link_libraries(objs_AP_SBusOut littlefs) #clean

#------------------------------------------------

set(objs_AP_Scheduler_SOURCES
    libraries/AP_Scheduler/PerfInfo.cpp)

set(objs_AP_Scheduler_INCLUDES
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

add_library(objs_AP_Scheduler ${objs_AP_Scheduler_SOURCES}) #2
target_include_directories(objs_AP_Scheduler PRIVATE ${objs_AP_Scheduler_INCLUDES})

set_target_properties(objs_AP_Scheduler PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler mavlink) #clean
target_link_libraries(objs_AP_Scheduler dronecan) #clean
target_link_libraries(objs_AP_Scheduler littlefs) #clean

#------------------------------------------------

set(objs_AP_SerialLED_SOURCES
    libraries/AP_SerialLED/AP_SerialLED.cpp)

set(objs_AP_SerialLED_INCLUDES
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

add_library(objs_AP_SerialLED ${objs_AP_SerialLED_SOURCES}) #2
target_include_directories(objs_AP_SerialLED PRIVATE ${objs_AP_SerialLED_INCLUDES})

set_target_properties(objs_AP_SerialLED PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_SerialLED mavlink) #clean
target_link_libraries(objs_AP_SerialLED dronecan) #clean
target_link_libraries(objs_AP_SerialLED littlefs) #clean

#------------------------------------------------

set(objs_AP_SerialManager_SOURCES
    libraries/AP_SerialManager/AP_SerialManager.cpp)

set(objs_AP_SerialManager_INCLUDES
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

add_library(objs_AP_SerialManager ${objs_AP_SerialManager_SOURCES}) #2
target_include_directories(objs_AP_SerialManager PRIVATE ${objs_AP_SerialManager_INCLUDES})

set_target_properties(objs_AP_SerialManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_SerialManager mavlink) #clean
target_link_libraries(objs_AP_SerialManager dronecan) #clean
target_link_libraries(objs_AP_SerialManager littlefs) #clean

#------------------------------------------------

set(objs_AP_ServoRelayEvents_SOURCES
    libraries/AP_ServoRelayEvents/AP_ServoRelayEvents.cpp)

set(objs_AP_ServoRelayEvents_INCLUDES
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

add_library(objs_AP_ServoRelayEvents ${objs_AP_ServoRelayEvents_SOURCES}) #2
target_include_directories(objs_AP_ServoRelayEvents PRIVATE ${objs_AP_ServoRelayEvents_INCLUDES})

set_target_properties(objs_AP_ServoRelayEvents PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ServoRelayEvents mavlink) #clean
target_link_libraries(objs_AP_ServoRelayEvents dronecan) #clean
target_link_libraries(objs_AP_ServoRelayEvents littlefs) #clean

#------------------------------------------------

set(objs_AP_Servo_Telem_SOURCES
    libraries/AP_Servo_Telem/AP_Servo_Telem.cpp)

set(objs_AP_Servo_Telem_INCLUDES
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

add_library(objs_AP_Servo_Telem ${objs_AP_Servo_Telem_SOURCES}) #2
target_include_directories(objs_AP_Servo_Telem PRIVATE ${objs_AP_Servo_Telem_INCLUDES})

set_target_properties(objs_AP_Servo_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Servo_Telem mavlink) #clean
target_link_libraries(objs_AP_Servo_Telem dronecan) #clean
target_link_libraries(objs_AP_Servo_Telem littlefs) #clean

#------------------------------------------------

set(objs_AP_SmartRTL_SOURCES
    libraries/AP_SmartRTL/AP_SmartRTL.cpp)

set(objs_AP_SmartRTL_INCLUDES
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

add_library(objs_AP_SmartRTL ${objs_AP_SmartRTL_SOURCES}) #2
target_include_directories(objs_AP_SmartRTL PRIVATE ${objs_AP_SmartRTL_INCLUDES})

set_target_properties(objs_AP_SmartRTL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_SmartRTL mavlink) #clean
target_link_libraries(objs_AP_SmartRTL dronecan) #clean
target_link_libraries(objs_AP_SmartRTL littlefs) #clean

#------------------------------------------------

set(objs_AP_Soaring_SOURCES
    libraries/AP_Soaring/AP_Soaring.cpp
    libraries/AP_Soaring/ExtendedKalmanFilter.cpp
    libraries/AP_Soaring/SpeedToFly.cpp
    libraries/AP_Soaring/Variometer.cpp)

set(objs_AP_Soaring_INCLUDES
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

add_library(objs_AP_Soaring ${objs_AP_Soaring_SOURCES}) #2
target_include_directories(objs_AP_Soaring PRIVATE ${objs_AP_Soaring_INCLUDES})

set_target_properties(objs_AP_Soaring PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Soaring mavlink) #clean
target_link_libraries(objs_AP_Soaring dronecan) #clean
target_link_libraries(objs_AP_Soaring littlefs) #clean

#------------------------------------------------

set(objs_AP_Stats_SOURCES
    libraries/AP_Stats/AP_Stats.cpp)

set(objs_AP_Stats_INCLUDES
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

add_library(objs_AP_Stats ${objs_AP_Stats_SOURCES}) #2
target_include_directories(objs_AP_Stats PRIVATE ${objs_AP_Stats_INCLUDES})

set_target_properties(objs_AP_Stats PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Stats mavlink) #clean
target_link_libraries(objs_AP_Stats dronecan) #clean
target_link_libraries(objs_AP_Stats littlefs) #clean

#------------------------------------------------

set(objs_AP_SurfaceDistance_SOURCES
    libraries/AP_SurfaceDistance/AP_SurfaceDistance.cpp)

set(objs_AP_SurfaceDistance_INCLUDES
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

add_library(objs_AP_SurfaceDistance ${objs_AP_SurfaceDistance_SOURCES}) #2
target_include_directories(objs_AP_SurfaceDistance PRIVATE ${objs_AP_SurfaceDistance_INCLUDES})

set_target_properties(objs_AP_SurfaceDistance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_SurfaceDistance mavlink) #clean
target_link_libraries(objs_AP_SurfaceDistance dronecan) #clean
target_link_libraries(objs_AP_SurfaceDistance littlefs) #clean

#------------------------------------------------

set(objs_AP_TECS_SOURCES
    libraries/AP_TECS/AP_TECS.cpp)

set(objs_AP_TECS_INCLUDES
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

add_library(objs_AP_TECS ${objs_AP_TECS_SOURCES}) #2
target_include_directories(objs_AP_TECS PRIVATE ${objs_AP_TECS_INCLUDES})

set_target_properties(objs_AP_TECS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TECS mavlink) #clean
target_link_libraries(objs_AP_TECS dronecan) #clean
target_link_libraries(objs_AP_TECS littlefs) #clean

#------------------------------------------------

set(objs_AP_TempCalibration_SOURCES
    libraries/AP_TempCalibration/AP_TempCalibration.cpp)

set(objs_AP_TempCalibration_INCLUDES
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

add_library(objs_AP_TempCalibration ${objs_AP_TempCalibration_SOURCES}) #2
target_include_directories(objs_AP_TempCalibration PRIVATE ${objs_AP_TempCalibration_INCLUDES})

set_target_properties(objs_AP_TempCalibration PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TempCalibration mavlink) #clean
target_link_libraries(objs_AP_TempCalibration dronecan) #clean
target_link_libraries(objs_AP_TempCalibration littlefs) #clean

#------------------------------------------------

set(objs_AP_TemperatureSensor_SOURCES
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

set(objs_AP_TemperatureSensor_INCLUDES
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

add_library(objs_AP_TemperatureSensor ${objs_AP_TemperatureSensor_SOURCES}) #2
target_include_directories(objs_AP_TemperatureSensor PRIVATE ${objs_AP_TemperatureSensor_INCLUDES})

set_target_properties(objs_AP_TemperatureSensor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor littlefs) #clean

#------------------------------------------------

set(objs_AP_Terrain_SOURCES
    libraries/AP_Terrain/TerrainGCS.cpp
    libraries/AP_Terrain/TerrainIO.cpp
    libraries/AP_Terrain/TerrainMission.cpp
    libraries/AP_Terrain/TerrainUtil.cpp)

set(objs_AP_Terrain_INCLUDES
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

add_library(objs_AP_Terrain ${objs_AP_Terrain_SOURCES}) #2
target_include_directories(objs_AP_Terrain PRIVATE ${objs_AP_Terrain_INCLUDES})

set_target_properties(objs_AP_Terrain PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain mavlink) #clean
target_link_libraries(objs_AP_Terrain dronecan) #clean
target_link_libraries(objs_AP_Terrain littlefs) #clean

#------------------------------------------------

set(objs_AP_Torqeedo_SOURCES
    libraries/AP_Torqeedo/AP_Torqeedo.cpp
    libraries/AP_Torqeedo/AP_Torqeedo_Backend.cpp
    libraries/AP_Torqeedo/AP_Torqeedo_Params.cpp
    libraries/AP_Torqeedo/AP_Torqeedo_TQBus.cpp)

set(objs_AP_Torqeedo_INCLUDES
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

add_library(objs_AP_Torqeedo ${objs_AP_Torqeedo_SOURCES}) #2
target_include_directories(objs_AP_Torqeedo PRIVATE ${objs_AP_Torqeedo_INCLUDES})

set_target_properties(objs_AP_Torqeedo PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Torqeedo mavlink) #clean
target_link_libraries(objs_AP_Torqeedo dronecan) #clean
target_link_libraries(objs_AP_Torqeedo littlefs) #clean

#------------------------------------------------

set(objs_AP_Tuning_SOURCES
    libraries/AP_Tuning/AP_Tuning.cpp)

set(objs_AP_Tuning_INCLUDES
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

add_library(objs_AP_Tuning ${objs_AP_Tuning_SOURCES}) #2
target_include_directories(objs_AP_Tuning PRIVATE ${objs_AP_Tuning_INCLUDES})

set_target_properties(objs_AP_Tuning PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Tuning mavlink) #clean
target_link_libraries(objs_AP_Tuning dronecan) #clean
target_link_libraries(objs_AP_Tuning littlefs) #clean

#------------------------------------------------

set(objs_AP_Vehicle_SOURCES
    libraries/AP_Vehicle/Parameters.cpp)

set(objs_AP_Vehicle_INCLUDES
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

add_library(objs_AP_Vehicle ${objs_AP_Vehicle_SOURCES}) #2
target_include_directories(objs_AP_Vehicle PRIVATE ${objs_AP_Vehicle_INCLUDES})

set_target_properties(objs_AP_Vehicle PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle mavlink) #clean
target_link_libraries(objs_AP_Vehicle dronecan) #clean
target_link_libraries(objs_AP_Vehicle littlefs) #clean

#------------------------------------------------

set(objs_AP_VideoTX_SOURCES
    libraries/AP_VideoTX/AP_SmartAudio.cpp
    libraries/AP_VideoTX/AP_Tramp.cpp
    libraries/AP_VideoTX/AP_VideoTX.cpp)

set(objs_AP_VideoTX_INCLUDES
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

add_library(objs_AP_VideoTX ${objs_AP_VideoTX_SOURCES}) #2
target_include_directories(objs_AP_VideoTX PRIVATE ${objs_AP_VideoTX_INCLUDES})

set_target_properties(objs_AP_VideoTX PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_VideoTX mavlink) #clean
target_link_libraries(objs_AP_VideoTX dronecan) #clean
target_link_libraries(objs_AP_VideoTX littlefs) #clean

#------------------------------------------------

set(objs_AP_VisualOdom_SOURCES
    libraries/AP_VisualOdom/AP_VisualOdom.cpp
    libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp
    libraries/AP_VisualOdom/AP_VisualOdom_IntelT265.cpp
    libraries/AP_VisualOdom/AP_VisualOdom_Logging.cpp
    libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp)

set(objs_AP_VisualOdom_INCLUDES
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

add_library(objs_AP_VisualOdom ${objs_AP_VisualOdom_SOURCES}) #2
target_include_directories(objs_AP_VisualOdom PRIVATE ${objs_AP_VisualOdom_INCLUDES})

set_target_properties(objs_AP_VisualOdom PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_VisualOdom mavlink) #clean
target_link_libraries(objs_AP_VisualOdom dronecan) #clean
target_link_libraries(objs_AP_VisualOdom littlefs) #clean

#------------------------------------------------

set(objs_AP_Volz_Protocol_SOURCES
    libraries/AP_Volz_Protocol/AP_Volz_Protocol.cpp)

set(objs_AP_Volz_Protocol_INCLUDES
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

add_library(objs_AP_Volz_Protocol ${objs_AP_Volz_Protocol_SOURCES}) #2
target_include_directories(objs_AP_Volz_Protocol PRIVATE ${objs_AP_Volz_Protocol_INCLUDES})

set_target_properties(objs_AP_Volz_Protocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Volz_Protocol mavlink) #clean
target_link_libraries(objs_AP_Volz_Protocol dronecan) #clean
target_link_libraries(objs_AP_Volz_Protocol littlefs) #clean

#------------------------------------------------

set(objs_AP_WheelEncoder_SOURCES
    libraries/AP_WheelEncoder/AP_WheelEncoder.cpp
    libraries/AP_WheelEncoder/AP_WheelRateControl.cpp
    libraries/AP_WheelEncoder/WheelEncoder_Backend.cpp
    libraries/AP_WheelEncoder/WheelEncoder_Quadrature.cpp
    libraries/AP_WheelEncoder/WheelEncoder_SITL_Quadrature.cpp)

set(objs_AP_WheelEncoder_INCLUDES
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

add_library(objs_AP_WheelEncoder ${objs_AP_WheelEncoder_SOURCES}) #2
target_include_directories(objs_AP_WheelEncoder PRIVATE ${objs_AP_WheelEncoder_INCLUDES})

set_target_properties(objs_AP_WheelEncoder PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_WheelEncoder mavlink) #clean
target_link_libraries(objs_AP_WheelEncoder dronecan) #clean
target_link_libraries(objs_AP_WheelEncoder littlefs) #clean

#------------------------------------------------

set(objs_AP_Winch_SOURCES
    libraries/AP_Winch/AP_Winch.cpp
    libraries/AP_Winch/AP_Winch_Backend.cpp
    libraries/AP_Winch/AP_Winch_Daiwa.cpp
    libraries/AP_Winch/AP_Winch_PWM.cpp)

set(objs_AP_Winch_INCLUDES
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

add_library(objs_AP_Winch ${objs_AP_Winch_SOURCES}) #2
target_include_directories(objs_AP_Winch PRIVATE ${objs_AP_Winch_INCLUDES})

set_target_properties(objs_AP_Winch PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Winch mavlink) #clean
target_link_libraries(objs_AP_Winch dronecan) #clean
target_link_libraries(objs_AP_Winch littlefs) #clean

#------------------------------------------------

set(objs_AP_WindVane_SOURCES
    libraries/AP_WindVane/AP_WindVane.cpp
    libraries/AP_WindVane/AP_WindVane_Airspeed.cpp
    libraries/AP_WindVane/AP_WindVane_Analog.cpp
    libraries/AP_WindVane/AP_WindVane_Backend.cpp
    libraries/AP_WindVane/AP_WindVane_Home.cpp
    libraries/AP_WindVane/AP_WindVane_ModernDevice.cpp
    libraries/AP_WindVane/AP_WindVane_NMEA.cpp
    libraries/AP_WindVane/AP_WindVane_RPM.cpp
    libraries/AP_WindVane/AP_WindVane_SITL.cpp)

set(objs_AP_WindVane_INCLUDES
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

add_library(objs_AP_WindVane ${objs_AP_WindVane_SOURCES}) #2
target_include_directories(objs_AP_WindVane PRIVATE ${objs_AP_WindVane_INCLUDES})

set_target_properties(objs_AP_WindVane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_WindVane mavlink) #clean
target_link_libraries(objs_AP_WindVane dronecan) #clean
target_link_libraries(objs_AP_WindVane littlefs) #clean

#------------------------------------------------

set(objs_AR_Motors_SOURCES
    libraries/AR_Motors/AP_MotorsUGV.cpp)

set(objs_AR_Motors_INCLUDES
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

add_library(objs_AR_Motors ${objs_AR_Motors_SOURCES}) #2
target_include_directories(objs_AR_Motors PRIVATE ${objs_AR_Motors_INCLUDES})

set_target_properties(objs_AR_Motors PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AR_Motors mavlink) #clean
target_link_libraries(objs_AR_Motors dronecan) #clean
target_link_libraries(objs_AR_Motors littlefs) #clean

#------------------------------------------------

set(objs_AR_WPNav_SOURCES
    libraries/AR_WPNav/AR_PivotTurn.cpp
    libraries/AR_WPNav/AR_WPNav.cpp
    libraries/AR_WPNav/AR_WPNav_OA.cpp)

set(objs_AR_WPNav_INCLUDES
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

add_library(objs_AR_WPNav ${objs_AR_WPNav_SOURCES}) #2
target_include_directories(objs_AR_WPNav PRIVATE ${objs_AR_WPNav_INCLUDES})

set_target_properties(objs_AR_WPNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AR_WPNav mavlink) #clean
target_link_libraries(objs_AR_WPNav dronecan) #clean
target_link_libraries(objs_AR_WPNav littlefs) #clean

#------------------------------------------------

set(objs_Filter_SOURCES
    libraries/Filter/AP_Filter.cpp
    libraries/Filter/AP_Filter_params.cpp
    libraries/Filter/AP_NotchFilter_params.cpp
    libraries/Filter/DerivativeFilter.cpp
    libraries/Filter/LowPassFilter.cpp
    libraries/Filter/LowPassFilter2p.cpp
    libraries/Filter/ModeFilter.cpp
    libraries/Filter/NotchFilter.cpp
    libraries/Filter/SlewLimiter.cpp)

set(objs_Filter_INCLUDES
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

add_library(objs_Filter ${objs_Filter_SOURCES}) #2
target_include_directories(objs_Filter PRIVATE ${objs_Filter_INCLUDES})

set_target_properties(objs_Filter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter mavlink) #clean
target_link_libraries(objs_Filter dronecan) #clean
target_link_libraries(objs_Filter littlefs) #clean

#------------------------------------------------

set(objs_GCS_MAVLink_SOURCES
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

set(objs_GCS_MAVLink_INCLUDES
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

add_library(objs_GCS_MAVLink ${objs_GCS_MAVLink_SOURCES}) #2
target_include_directories(objs_GCS_MAVLink PRIVATE ${objs_GCS_MAVLink_INCLUDES})

set_target_properties(objs_GCS_MAVLink PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink mavlink) #clean
target_link_libraries(objs_GCS_MAVLink dronecan) #clean
target_link_libraries(objs_GCS_MAVLink littlefs) #clean

#------------------------------------------------

set(objs_PID_SOURCES
    libraries/PID/PID.cpp)

set(objs_PID_INCLUDES
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

add_library(objs_PID ${objs_PID_SOURCES}) #2
target_include_directories(objs_PID PRIVATE ${objs_PID_INCLUDES})

set_target_properties(objs_PID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_PID mavlink) #clean
target_link_libraries(objs_PID dronecan) #clean
target_link_libraries(objs_PID littlefs) #clean

#------------------------------------------------

set(objs_RC_Channel_SOURCES
    libraries/RC_Channel/RC_Channels.cpp)

set(objs_RC_Channel_INCLUDES
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

add_library(objs_RC_Channel ${objs_RC_Channel_SOURCES}) #2
target_include_directories(objs_RC_Channel PRIVATE ${objs_RC_Channel_INCLUDES})

set_target_properties(objs_RC_Channel PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel mavlink) #clean
target_link_libraries(objs_RC_Channel dronecan) #clean
target_link_libraries(objs_RC_Channel littlefs) #clean

#------------------------------------------------

set(objs_SRV_Channel_SOURCES
    libraries/SRV_Channel/SRV_Channel.cpp
    libraries/SRV_Channel/SRV_Channel_aux.cpp
    libraries/SRV_Channel/SRV_Channels.cpp)

set(objs_SRV_Channel_INCLUDES
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

add_library(objs_SRV_Channel ${objs_SRV_Channel_SOURCES}) #2
target_include_directories(objs_SRV_Channel PRIVATE ${objs_SRV_Channel_INCLUDES})

set_target_properties(objs_SRV_Channel PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SRV_Channel mavlink) #clean
target_link_libraries(objs_SRV_Channel dronecan) #clean
target_link_libraries(objs_SRV_Channel littlefs) #clean

#------------------------------------------------

set(objs_AP_HAL_SOURCES
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

set(objs_AP_HAL_INCLUDES
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

add_library(objs_AP_HAL ${objs_AP_HAL_SOURCES}) #2
target_include_directories(objs_AP_HAL PRIVATE ${objs_AP_HAL_INCLUDES})

set_target_properties(objs_AP_HAL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL mavlink) #clean
target_link_libraries(objs_AP_HAL dronecan) #clean
target_link_libraries(objs_AP_HAL littlefs) #clean

#------------------------------------------------

set(objs_AP_HAL_Empty_SOURCES
    libraries/AP_HAL_Empty/AnalogIn.cpp
    libraries/AP_HAL_Empty/GPIO.cpp
    libraries/AP_HAL_Empty/HAL_Empty_Class.cpp
    libraries/AP_HAL_Empty/RCInput.cpp
    libraries/AP_HAL_Empty/RCOutput.cpp
    libraries/AP_HAL_Empty/Semaphores.cpp
    libraries/AP_HAL_Empty/Storage.cpp
    libraries/AP_HAL_Empty/UARTDriver.cpp)

set(objs_AP_HAL_Empty_INCLUDES
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

add_library(objs_AP_HAL_Empty ${objs_AP_HAL_Empty_SOURCES}) #2
target_include_directories(objs_AP_HAL_Empty PRIVATE ${objs_AP_HAL_Empty_INCLUDES})

set_target_properties(objs_AP_HAL_Empty PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_Empty mavlink) #clean
target_link_libraries(objs_AP_HAL_Empty dronecan) #clean
target_link_libraries(objs_AP_HAL_Empty littlefs) #clean

#------------------------------------------------

set(objs_AP_PiccoloCAN_piccolo_protocol_SOURCES
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

set(objs_AP_PiccoloCAN_piccolo_protocol_INCLUDES
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

add_library(objs_AP_PiccoloCAN_piccolo_protocol ${objs_AP_PiccoloCAN_piccolo_protocol_SOURCES}) #2
target_include_directories(objs_AP_PiccoloCAN_piccolo_protocol PRIVATE ${objs_AP_PiccoloCAN_piccolo_protocol_INCLUDES})

set_target_properties(objs_AP_PiccoloCAN_piccolo_protocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_PiccoloCAN_piccolo_protocol mavlink) #clean
target_link_libraries(objs_AP_PiccoloCAN_piccolo_protocol dronecan) #clean
target_link_libraries(objs_AP_PiccoloCAN_piccolo_protocol littlefs) #clean

#------------------------------------------------

set(objs_AP_HAL_SITL_SOURCES
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

set(objs_AP_HAL_SITL_INCLUDES
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

add_library(objs_AP_HAL_SITL ${objs_AP_HAL_SITL_SOURCES}) #2
target_include_directories(objs_AP_HAL_SITL PRIVATE ${objs_AP_HAL_SITL_INCLUDES})

set_target_properties(objs_AP_HAL_SITL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL littlefs) #clean

#------------------------------------------------

set(objs_SITL_SOURCES
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

set(objs_SITL_INCLUDES
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

add_library(objs_SITL ${objs_SITL_SOURCES}) #2
target_include_directories(objs_SITL PRIVATE ${objs_SITL_INCLUDES})

set_target_properties(objs_SITL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL mavlink) #clean
target_link_libraries(objs_SITL dronecan) #clean
target_link_libraries(objs_SITL littlefs) #clean

#------------------------------------------------

set(objs_AP_Scripting_SOURCES
    libraries/AP_Scripting/AP_Scripting.cpp
    libraries/AP_Scripting/AP_Scripting_CANSensor.cpp
    libraries/AP_Scripting/AP_Scripting_SerialAccess.cpp
    libraries/AP_Scripting/AP_Scripting_SerialDevice.cpp)

set(objs_AP_Scripting_INCLUDES
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

add_library(objs_AP_Scripting ${objs_AP_Scripting_SOURCES}) #2
target_include_directories(objs_AP_Scripting PRIVATE ${objs_AP_Scripting_INCLUDES})

set_target_properties(objs_AP_Scripting PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting mavlink) #clean
target_link_libraries(objs_AP_Scripting dronecan) #clean
target_link_libraries(objs_AP_Scripting littlefs) #clean

#------------------------------------------------

set(objs_AP_Scripting_lua_src_SOURCES
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

set(objs_AP_Scripting_lua_src_INCLUDES
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

add_library(objs_AP_Scripting_lua_src ${objs_AP_Scripting_lua_src_SOURCES}) #2
target_include_directories(objs_AP_Scripting_lua_src PRIVATE ${objs_AP_Scripting_lua_src_INCLUDES})

set_target_properties(objs_AP_Scripting_lua_src PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_lua_src mavlink) #clean
target_link_libraries(objs_AP_Scripting_lua_src dronecan) #clean
target_link_libraries(objs_AP_Scripting_lua_src littlefs) #clean

#------------------------------------------------

set(objs_AP_AHRS_AntennaTracker_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs_AP_AHRS_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_AHRS_AntennaTracker ${objs_AP_AHRS_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_AHRS_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_AHRS_AntennaTracker PRIVATE ${objs_AP_AHRS_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_AHRS_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_AHRS_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_AHRS_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Airspeed_AntennaTracker_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs_AP_Airspeed_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Airspeed_AntennaTracker ${objs_AP_Airspeed_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Airspeed_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Airspeed_AntennaTracker PRIVATE ${objs_AP_Airspeed_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Airspeed_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Airspeed_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Airspeed_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Baro_AntennaTracker_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs_AP_Baro_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Baro_AntennaTracker ${objs_AP_Baro_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Baro_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Baro_AntennaTracker PRIVATE ${objs_AP_Baro_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Baro_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Baro_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Baro_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_BattMonitor_AntennaTracker_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs_AP_BattMonitor_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_BattMonitor_AntennaTracker ${objs_AP_BattMonitor_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_BattMonitor_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_BattMonitor_AntennaTracker PRIVATE ${objs_AP_BattMonitor_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_BattMonitor_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_BoardConfig_AntennaTracker_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs_AP_BoardConfig_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_BoardConfig_AntennaTracker ${objs_AP_BoardConfig_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_BoardConfig_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_BoardConfig_AntennaTracker PRIVATE ${objs_AP_BoardConfig_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_BoardConfig_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_HAL_AntennaTracker_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs_AP_HAL_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_HAL_AntennaTracker ${objs_AP_HAL_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_HAL_AntennaTracker PRIVATE ${objs_AP_HAL_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_HAL_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_HAL_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_HAL_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_InertialSensor_AntennaTracker_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs_AP_InertialSensor_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_InertialSensor_AntennaTracker ${objs_AP_InertialSensor_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_InertialSensor_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_InertialSensor_AntennaTracker PRIVATE ${objs_AP_InertialSensor_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_InertialSensor_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Math_AntennaTracker_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs_AP_Math_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Math_AntennaTracker ${objs_AP_Math_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Math_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Math_AntennaTracker PRIVATE ${objs_AP_Math_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Math_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Math_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Math_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Mission_AntennaTracker_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs_AP_Mission_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Mission_AntennaTracker ${objs_AP_Mission_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Mission_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Mission_AntennaTracker PRIVATE ${objs_AP_Mission_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Mission_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Mission_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Mission_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_DAL_AntennaTracker_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs_AP_DAL_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_DAL_AntennaTracker ${objs_AP_DAL_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_DAL_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_DAL_AntennaTracker PRIVATE ${objs_AP_DAL_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_DAL_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_DAL_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_DAL_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_NavEKF2_AntennaTracker_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs_AP_NavEKF2_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_NavEKF2_AntennaTracker ${objs_AP_NavEKF2_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF2_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_NavEKF2_AntennaTracker PRIVATE ${objs_AP_NavEKF2_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_NavEKF2_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_NavEKF3_AntennaTracker_SOURCES
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

set(objs_AP_NavEKF3_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_NavEKF3_AntennaTracker ${objs_AP_NavEKF3_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF3_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_NavEKF3_AntennaTracker PRIVATE ${objs_AP_NavEKF3_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_NavEKF3_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Rally_AntennaTracker_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs_AP_Rally_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Rally_AntennaTracker ${objs_AP_Rally_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Rally_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Rally_AntennaTracker PRIVATE ${objs_AP_Rally_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Rally_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Rally_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Rally_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Scheduler_AntennaTracker_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs_AP_Scheduler_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Scheduler_AntennaTracker ${objs_AP_Scheduler_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Scheduler_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Scheduler_AntennaTracker PRIVATE ${objs_AP_Scheduler_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Scheduler_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Scheduler_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Scheduler_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Terrain_AntennaTracker_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs_AP_Terrain_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Terrain_AntennaTracker ${objs_AP_Terrain_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Terrain_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Terrain_AntennaTracker PRIVATE ${objs_AP_Terrain_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Terrain_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Terrain_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Terrain_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Vehicle_AntennaTracker_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs_AP_Vehicle_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Vehicle_AntennaTracker ${objs_AP_Vehicle_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Vehicle_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Vehicle_AntennaTracker PRIVATE ${objs_AP_Vehicle_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Vehicle_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Vehicle_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Vehicle_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Logger_AntennaTracker_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs_AP_Logger_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Logger_AntennaTracker ${objs_AP_Logger_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Logger_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Logger_AntennaTracker PRIVATE ${objs_AP_Logger_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Logger_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Logger_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Logger_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_Filter_AntennaTracker_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs_Filter_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_Filter_AntennaTracker ${objs_Filter_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_Filter_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_Filter_AntennaTracker PRIVATE ${objs_Filter_AntennaTracker_INCLUDES})

set_target_properties(objs_Filter_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_AntennaTracker mavlink) #clean
target_link_libraries(objs_Filter_AntennaTracker dronecan) #clean
target_link_libraries(objs_Filter_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_GCS_MAVLink_AntennaTracker_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs_GCS_MAVLink_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_GCS_MAVLink_AntennaTracker ${objs_GCS_MAVLink_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_GCS_MAVLink_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_GCS_MAVLink_AntennaTracker PRIVATE ${objs_GCS_MAVLink_AntennaTracker_INCLUDES})

set_target_properties(objs_GCS_MAVLink_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_AntennaTracker mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_AntennaTracker dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_RC_Channel_AntennaTracker_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs_RC_Channel_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_RC_Channel_AntennaTracker ${objs_RC_Channel_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_RC_Channel_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_RC_Channel_AntennaTracker PRIVATE ${objs_RC_Channel_AntennaTracker_INCLUDES})

set_target_properties(objs_RC_Channel_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_AntennaTracker mavlink) #clean
target_link_libraries(objs_RC_Channel_AntennaTracker dronecan) #clean
target_link_libraries(objs_RC_Channel_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_StorageManager_AntennaTracker_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs_StorageManager_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_StorageManager_AntennaTracker ${objs_StorageManager_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_StorageManager_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_StorageManager_AntennaTracker PRIVATE ${objs_StorageManager_AntennaTracker_INCLUDES})

set_target_properties(objs_StorageManager_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_AntennaTracker mavlink) #clean
target_link_libraries(objs_StorageManager_AntennaTracker dronecan) #clean
target_link_libraries(objs_StorageManager_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Frsky_Telem_AntennaTracker_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs_AP_Frsky_Telem_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Frsky_Telem_AntennaTracker ${objs_AP_Frsky_Telem_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Frsky_Telem_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Frsky_Telem_AntennaTracker PRIVATE ${objs_AP_Frsky_Telem_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Frsky_Telem_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_IBus_Telem_AntennaTracker_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs_AP_IBus_Telem_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_IBus_Telem_AntennaTracker ${objs_AP_IBus_Telem_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_IBus_Telem_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_IBus_Telem_AntennaTracker PRIVATE ${objs_AP_IBus_Telem_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_IBus_Telem_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Relay_AntennaTracker_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs_AP_Relay_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Relay_AntennaTracker ${objs_AP_Relay_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Relay_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Relay_AntennaTracker PRIVATE ${objs_AP_Relay_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Relay_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Relay_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Relay_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_RCProtocol_AntennaTracker_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs_AP_RCProtocol_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_RCProtocol_AntennaTracker ${objs_AP_RCProtocol_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_RCProtocol_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_RCProtocol_AntennaTracker PRIVATE ${objs_AP_RCProtocol_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_RCProtocol_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_BLHeli_AntennaTracker_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs_AP_BLHeli_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_BLHeli_AntennaTracker ${objs_AP_BLHeli_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_BLHeli_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_BLHeli_AntennaTracker PRIVATE ${objs_AP_BLHeli_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_BLHeli_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_BLHeli_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_BLHeli_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Proximity_AntennaTracker_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs_AP_Proximity_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Proximity_AntennaTracker ${objs_AP_Proximity_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Proximity_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Proximity_AntennaTracker PRIVATE ${objs_AP_Proximity_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Proximity_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Proximity_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Proximity_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AC_Fence_AntennaTracker_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs_AC_Fence_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AC_Fence_AntennaTracker ${objs_AC_Fence_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AC_Fence_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AC_Fence_AntennaTracker PRIVATE ${objs_AC_Fence_AntennaTracker_INCLUDES})

set_target_properties(objs_AC_Fence_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_AntennaTracker mavlink) #clean
target_link_libraries(objs_AC_Fence_AntennaTracker dronecan) #clean
target_link_libraries(objs_AC_Fence_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AC_Avoidance_AntennaTracker_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs_AC_Avoidance_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AC_Avoidance_AntennaTracker ${objs_AC_Avoidance_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AC_Avoidance_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AC_Avoidance_AntennaTracker PRIVATE ${objs_AC_Avoidance_AntennaTracker_INCLUDES})

set_target_properties(objs_AC_Avoidance_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_AntennaTracker mavlink) #clean
target_link_libraries(objs_AC_Avoidance_AntennaTracker dronecan) #clean
target_link_libraries(objs_AC_Avoidance_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_LandingGear_AntennaTracker_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs_AP_LandingGear_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_LandingGear_AntennaTracker ${objs_AP_LandingGear_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_LandingGear_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_LandingGear_AntennaTracker PRIVATE ${objs_AP_LandingGear_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_LandingGear_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_LandingGear_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_LandingGear_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_OSD_AntennaTracker_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs_AP_OSD_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_OSD_AntennaTracker ${objs_AP_OSD_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_OSD_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_OSD_AntennaTracker PRIVATE ${objs_AP_OSD_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_OSD_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_OSD_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_OSD_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Filesystem_AntennaTracker_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs_AP_Filesystem_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Filesystem_AntennaTracker ${objs_AP_Filesystem_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Filesystem_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Filesystem_AntennaTracker PRIVATE ${objs_AP_Filesystem_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Filesystem_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Filesystem_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Filesystem_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_ADSB_AntennaTracker_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs_AP_ADSB_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_ADSB_AntennaTracker ${objs_AP_ADSB_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_ADSB_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_ADSB_AntennaTracker PRIVATE ${objs_AP_ADSB_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_ADSB_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_ADSB_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_ADSB_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_ADSB_sagetech-sdk_SOURCES
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

set(objs_AP_ADSB_sagetech-sdk_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_ADSB_sagetech-sdk ${objs_AP_ADSB_sagetech-sdk_SOURCES}) #2
target_include_directories(objs_AP_ADSB_sagetech-sdk PRIVATE ${objs_AP_ADSB_sagetech-sdk_INCLUDES})

set_target_properties(objs_AP_ADSB_sagetech-sdk PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_sagetech-sdk mavlink) #clean
target_link_libraries(objs_AP_ADSB_sagetech-sdk dronecan) #clean
target_link_libraries(objs_AP_ADSB_sagetech-sdk littlefs) #clean

#------------------------------------------------

set(objs_AP_ESC_Telem_AntennaTracker_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs_AP_ESC_Telem_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_ESC_Telem_AntennaTracker ${objs_AP_ESC_Telem_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_ESC_Telem_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_ESC_Telem_AntennaTracker PRIVATE ${objs_AP_ESC_Telem_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_ESC_Telem_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_GyroFFT_AntennaTracker_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs_AP_GyroFFT_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_GyroFFT_AntennaTracker ${objs_AP_GyroFFT_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_GyroFFT_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_GyroFFT_AntennaTracker PRIVATE ${objs_AP_GyroFFT_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_GyroFFT_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_RCTelemetry_AntennaTracker_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs_AP_RCTelemetry_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_RCTelemetry_AntennaTracker ${objs_AP_RCTelemetry_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_RCTelemetry_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_RCTelemetry_AntennaTracker PRIVATE ${objs_AP_RCTelemetry_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_RCTelemetry_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_ExternalAHRS_AntennaTracker_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs_AP_ExternalAHRS_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_ExternalAHRS_AntennaTracker ${objs_AP_ExternalAHRS_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_ExternalAHRS_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_ExternalAHRS_AntennaTracker PRIVATE ${objs_AP_ExternalAHRS_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_ExternalAHRS_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_TemperatureSensor_AntennaTracker_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs_AP_TemperatureSensor_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_TemperatureSensor_AntennaTracker ${objs_AP_TemperatureSensor_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_TemperatureSensor_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_TemperatureSensor_AntennaTracker PRIVATE ${objs_AP_TemperatureSensor_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_TemperatureSensor_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_AIS_AntennaTracker_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs_AP_AIS_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_AIS_AntennaTracker ${objs_AP_AIS_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_AIS_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_AIS_AntennaTracker PRIVATE ${objs_AP_AIS_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_AIS_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_AIS_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_AIS_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Arming_AntennaTracker_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs_AP_Arming_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Arming_AntennaTracker ${objs_AP_Arming_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Arming_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Arming_AntennaTracker PRIVATE ${objs_AP_Arming_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Arming_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Arming_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Arming_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Follow_AntennaTracker_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs_AP_Follow_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Follow_AntennaTracker ${objs_AP_Follow_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Follow_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Follow_AntennaTracker PRIVATE ${objs_AP_Follow_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Follow_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Follow_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Follow_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_HAL_SITL_AntennaTracker_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs_AP_HAL_SITL_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_HAL_SITL_AntennaTracker ${objs_AP_HAL_SITL_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_SITL_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_HAL_SITL_AntennaTracker PRIVATE ${objs_AP_HAL_SITL_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_HAL_SITL_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_SITL_AntennaTracker_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs_SITL_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_SITL_AntennaTracker ${objs_SITL_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_SITL_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_SITL_AntennaTracker PRIVATE ${objs_SITL_AntennaTracker_INCLUDES})

set_target_properties(objs_SITL_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_AntennaTracker mavlink) #clean
target_link_libraries(objs_SITL_AntennaTracker dronecan) #clean
target_link_libraries(objs_SITL_AntennaTracker littlefs) #clean

#------------------------------------------------

set(objs_AP_Scripting_AntennaTracker_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs_AP_Scripting_AntennaTracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Scripting_AntennaTracker ${objs_AP_Scripting_AntennaTracker_SOURCES}) #2
target_compile_definitions(objs_AP_Scripting_AntennaTracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Scripting_AntennaTracker PRIVATE ${objs_AP_Scripting_AntennaTracker_INCLUDES})

set_target_properties(objs_AP_Scripting_AntennaTracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_AntennaTracker mavlink) #clean
target_link_libraries(objs_AP_Scripting_AntennaTracker dronecan) #clean
target_link_libraries(objs_AP_Scripting_AntennaTracker littlefs) #clean

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


target_link_libraries(AntennaTracker_libs objs_AP_AccelCal_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_ADC_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_AHRS_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Airspeed_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Baro_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_BattMonitor_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_BoardConfig_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Camera_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Common_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Compass_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Declination_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_GPS_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_GSOF_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_HAL_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_HAL_Empty_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_DDS_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_InertialSensor_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Math_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Mission_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_DAL_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_NavEKF_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_NavEKF2_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_NavEKF3_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Notify_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_OpticalFlow_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Param_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Rally_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_LightWareSerial_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RangeFinder_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Scheduler_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_SerialManager_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Terrain_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Vehicle_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_InternalError_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Logger_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_Filter_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_GCS_MAVLink_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_RC_Channel_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_SRV_Channel_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_StorageManager_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Tuning_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RPM_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RSSI_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Mount_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Module_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Button_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_ICEngine_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Networking_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Frsky_Telem_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_IBus_Telem_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_FlashStorage_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Relay_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_ServoRelayEvents_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Volz_Protocol_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_SBusOut_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_IOMCU_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Parachute_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RAMTRON_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RCProtocol_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Radio_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_TempCalibration_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_VisualOdom_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_BLHeli_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_ROMFS_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Proximity_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Gripper_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RTC_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AC_Sprayer_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AC_Fence_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AC_Avoidance_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_LandingGear_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RobotisServo_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_NMEA_Output_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_OSD_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Filesystem_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_ADSB_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AC_PID_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_SerialLED_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_EFI_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Hott_Telem_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_ESC_Telem_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Servo_Telem_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Stats_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_GyroFFT_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RCTelemetry_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Generator_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_MSP_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_OLC_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_WheelEncoder_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_ExternalAHRS_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_VideoTX_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_FETtecOneWire_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_TemperatureSensor_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Torqeedo_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_CustomRotations_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_AIS_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_OpenDroneID_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_CheckFirmware_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_ExternalControl_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_JSON_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Beacon_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Arming_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_RCMapper_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_MultiHeap_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Follow_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_CANManager_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_KDECAN_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_PiccoloCAN_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_DroneCAN_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_HAL_SITL_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_CSVReader_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_SITL_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AP_Scripting_AntennaTracker) #stripped
target_link_libraries(AntennaTracker_libs objs_AC_CustomControl_AntennaTracker) #stripped

#------------------------------------------------

set(bin_antennatracker_SOURCES
    GCS_MAVLink_Tracker.cpp
    GCS_Tracker.cpp
    Log.cpp
    Parameters.cpp
    RC_Channel_Tracker.cpp
    Tracker.cpp
    mode.cpp
    mode_auto.cpp
    mode_guided.cpp
    mode_manual.cpp
    mode_scan.cpp
    mode_servotest.cpp
    radio.cpp
    sensors.cpp
    servos.cpp
    system.cpp
    tracking.cpp)

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
target_compile_definitions(bin_antennatracker PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(bin_antennatracker PRIVATE ${bin_antennatracker_INCLUDES})


target_link_libraries(bin_antennatracker AntennaTracker_libs) #clean

#------------------------------------------------

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

add_library(objs_AP_AHRS_ArduCopter ${objs_AP_AHRS_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_AHRS_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_AHRS_ArduCopter PRIVATE ${objs_AP_AHRS_ArduCopter_INCLUDES})

set_target_properties(objs_AP_AHRS_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_AHRS_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_AHRS_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Airspeed_ArduCopter ${objs_AP_Airspeed_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Airspeed_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Airspeed_ArduCopter PRIVATE ${objs_AP_Airspeed_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Airspeed_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Airspeed_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Airspeed_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Baro_ArduCopter ${objs_AP_Baro_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Baro_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Baro_ArduCopter PRIVATE ${objs_AP_Baro_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Baro_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Baro_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Baro_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BattMonitor_ArduCopter ${objs_AP_BattMonitor_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_BattMonitor_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_BattMonitor_ArduCopter PRIVATE ${objs_AP_BattMonitor_ArduCopter_INCLUDES})

set_target_properties(objs_AP_BattMonitor_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BoardConfig_ArduCopter ${objs_AP_BoardConfig_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_BoardConfig_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_BoardConfig_ArduCopter PRIVATE ${objs_AP_BoardConfig_ArduCopter_INCLUDES})

set_target_properties(objs_AP_BoardConfig_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_HAL_ArduCopter ${objs_AP_HAL_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_HAL_ArduCopter PRIVATE ${objs_AP_HAL_ArduCopter_INCLUDES})

set_target_properties(objs_AP_HAL_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_HAL_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_HAL_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_InertialSensor_ArduCopter ${objs_AP_InertialSensor_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_InertialSensor_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_InertialSensor_ArduCopter PRIVATE ${objs_AP_InertialSensor_ArduCopter_INCLUDES})

set_target_properties(objs_AP_InertialSensor_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Math_ArduCopter ${objs_AP_Math_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Math_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Math_ArduCopter PRIVATE ${objs_AP_Math_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Math_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Math_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Math_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Mission_ArduCopter ${objs_AP_Mission_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Mission_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Mission_ArduCopter PRIVATE ${objs_AP_Mission_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Mission_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Mission_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Mission_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_DAL_ArduCopter ${objs_AP_DAL_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_DAL_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_DAL_ArduCopter PRIVATE ${objs_AP_DAL_ArduCopter_INCLUDES})

set_target_properties(objs_AP_DAL_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_DAL_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_DAL_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_NavEKF2_ArduCopter ${objs_AP_NavEKF2_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF2_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_NavEKF2_ArduCopter PRIVATE ${objs_AP_NavEKF2_ArduCopter_INCLUDES})

set_target_properties(objs_AP_NavEKF2_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_NavEKF3_ArduCopter ${objs_AP_NavEKF3_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF3_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_NavEKF3_ArduCopter PRIVATE ${objs_AP_NavEKF3_ArduCopter_INCLUDES})

set_target_properties(objs_AP_NavEKF3_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Rally_ArduCopter ${objs_AP_Rally_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Rally_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Rally_ArduCopter PRIVATE ${objs_AP_Rally_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Rally_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Rally_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Rally_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Scheduler_ArduCopter ${objs_AP_Scheduler_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Scheduler_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Scheduler_ArduCopter PRIVATE ${objs_AP_Scheduler_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Scheduler_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Scheduler_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Scheduler_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Terrain_ArduCopter ${objs_AP_Terrain_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Terrain_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Terrain_ArduCopter PRIVATE ${objs_AP_Terrain_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Terrain_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Terrain_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Terrain_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Vehicle_ArduCopter ${objs_AP_Vehicle_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Vehicle_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Vehicle_ArduCopter PRIVATE ${objs_AP_Vehicle_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Vehicle_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Vehicle_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Vehicle_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Logger_ArduCopter ${objs_AP_Logger_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Logger_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Logger_ArduCopter PRIVATE ${objs_AP_Logger_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Logger_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Logger_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Logger_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_Filter_ArduCopter ${objs_Filter_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_Filter_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_Filter_ArduCopter PRIVATE ${objs_Filter_ArduCopter_INCLUDES})

set_target_properties(objs_Filter_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_ArduCopter mavlink) #clean
target_link_libraries(objs_Filter_ArduCopter dronecan) #clean
target_link_libraries(objs_Filter_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_GCS_MAVLink_ArduCopter ${objs_GCS_MAVLink_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_GCS_MAVLink_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_GCS_MAVLink_ArduCopter PRIVATE ${objs_GCS_MAVLink_ArduCopter_INCLUDES})

set_target_properties(objs_GCS_MAVLink_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_ArduCopter mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_ArduCopter dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_RC_Channel_ArduCopter ${objs_RC_Channel_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_RC_Channel_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_RC_Channel_ArduCopter PRIVATE ${objs_RC_Channel_ArduCopter_INCLUDES})

set_target_properties(objs_RC_Channel_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_ArduCopter mavlink) #clean
target_link_libraries(objs_RC_Channel_ArduCopter dronecan) #clean
target_link_libraries(objs_RC_Channel_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_StorageManager_ArduCopter ${objs_StorageManager_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_StorageManager_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_StorageManager_ArduCopter PRIVATE ${objs_StorageManager_ArduCopter_INCLUDES})

set_target_properties(objs_StorageManager_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_ArduCopter mavlink) #clean
target_link_libraries(objs_StorageManager_ArduCopter dronecan) #clean
target_link_libraries(objs_StorageManager_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Frsky_Telem_ArduCopter ${objs_AP_Frsky_Telem_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Frsky_Telem_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Frsky_Telem_ArduCopter PRIVATE ${objs_AP_Frsky_Telem_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Frsky_Telem_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_IBus_Telem_ArduCopter ${objs_AP_IBus_Telem_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_IBus_Telem_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_IBus_Telem_ArduCopter PRIVATE ${objs_AP_IBus_Telem_ArduCopter_INCLUDES})

set_target_properties(objs_AP_IBus_Telem_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Relay_ArduCopter ${objs_AP_Relay_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Relay_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Relay_ArduCopter PRIVATE ${objs_AP_Relay_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Relay_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Relay_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Relay_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_RCProtocol_ArduCopter ${objs_AP_RCProtocol_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_RCProtocol_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_RCProtocol_ArduCopter PRIVATE ${objs_AP_RCProtocol_ArduCopter_INCLUDES})

set_target_properties(objs_AP_RCProtocol_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BLHeli_ArduCopter ${objs_AP_BLHeli_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_BLHeli_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_BLHeli_ArduCopter PRIVATE ${objs_AP_BLHeli_ArduCopter_INCLUDES})

set_target_properties(objs_AP_BLHeli_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_BLHeli_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_BLHeli_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Proximity_ArduCopter ${objs_AP_Proximity_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Proximity_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Proximity_ArduCopter PRIVATE ${objs_AP_Proximity_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Proximity_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Proximity_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Proximity_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_Fence_ArduCopter ${objs_AC_Fence_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AC_Fence_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AC_Fence_ArduCopter PRIVATE ${objs_AC_Fence_ArduCopter_INCLUDES})

set_target_properties(objs_AC_Fence_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_Fence_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_Fence_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_Avoidance_ArduCopter ${objs_AC_Avoidance_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AC_Avoidance_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AC_Avoidance_ArduCopter PRIVATE ${objs_AC_Avoidance_ArduCopter_INCLUDES})

set_target_properties(objs_AC_Avoidance_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_Avoidance_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_Avoidance_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_LandingGear_ArduCopter ${objs_AP_LandingGear_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_LandingGear_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_LandingGear_ArduCopter PRIVATE ${objs_AP_LandingGear_ArduCopter_INCLUDES})

set_target_properties(objs_AP_LandingGear_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_LandingGear_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_LandingGear_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_OSD_ArduCopter ${objs_AP_OSD_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_OSD_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_OSD_ArduCopter PRIVATE ${objs_AP_OSD_ArduCopter_INCLUDES})

set_target_properties(objs_AP_OSD_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_OSD_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_OSD_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Filesystem_ArduCopter ${objs_AP_Filesystem_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Filesystem_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Filesystem_ArduCopter PRIVATE ${objs_AP_Filesystem_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Filesystem_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Filesystem_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Filesystem_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ADSB_ArduCopter ${objs_AP_ADSB_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_ADSB_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_ADSB_ArduCopter PRIVATE ${objs_AP_ADSB_ArduCopter_INCLUDES})

set_target_properties(objs_AP_ADSB_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_ADSB_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_ADSB_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ESC_Telem_ArduCopter ${objs_AP_ESC_Telem_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_ESC_Telem_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_ESC_Telem_ArduCopter PRIVATE ${objs_AP_ESC_Telem_ArduCopter_INCLUDES})

set_target_properties(objs_AP_ESC_Telem_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_GyroFFT_ArduCopter ${objs_AP_GyroFFT_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_GyroFFT_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_GyroFFT_ArduCopter PRIVATE ${objs_AP_GyroFFT_ArduCopter_INCLUDES})

set_target_properties(objs_AP_GyroFFT_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_RCTelemetry_ArduCopter ${objs_AP_RCTelemetry_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_RCTelemetry_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_RCTelemetry_ArduCopter PRIVATE ${objs_AP_RCTelemetry_ArduCopter_INCLUDES})

set_target_properties(objs_AP_RCTelemetry_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ExternalAHRS_ArduCopter ${objs_AP_ExternalAHRS_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_ExternalAHRS_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_ExternalAHRS_ArduCopter PRIVATE ${objs_AP_ExternalAHRS_ArduCopter_INCLUDES})

set_target_properties(objs_AP_ExternalAHRS_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_TemperatureSensor_ArduCopter ${objs_AP_TemperatureSensor_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_TemperatureSensor_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_TemperatureSensor_ArduCopter PRIVATE ${objs_AP_TemperatureSensor_ArduCopter_INCLUDES})

set_target_properties(objs_AP_TemperatureSensor_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_AIS_ArduCopter ${objs_AP_AIS_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_AIS_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_AIS_ArduCopter PRIVATE ${objs_AP_AIS_ArduCopter_INCLUDES})

set_target_properties(objs_AP_AIS_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_AIS_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_AIS_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Arming_ArduCopter ${objs_AP_Arming_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Arming_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Arming_ArduCopter PRIVATE ${objs_AP_Arming_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Arming_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Arming_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Arming_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Follow_ArduCopter ${objs_AP_Follow_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Follow_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Follow_ArduCopter PRIVATE ${objs_AP_Follow_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Follow_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Follow_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Follow_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_AttitudeControl_ArduCopter ${objs_AC_AttitudeControl_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AC_AttitudeControl_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AC_AttitudeControl_ArduCopter PRIVATE ${objs_AC_AttitudeControl_ArduCopter_INCLUDES})

set_target_properties(objs_AC_AttitudeControl_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_PrecLand_ArduCopter ${objs_AC_PrecLand_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AC_PrecLand_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AC_PrecLand_ArduCopter PRIVATE ${objs_AC_PrecLand_ArduCopter_INCLUDES})

set_target_properties(objs_AC_PrecLand_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_PrecLand_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_PrecLand_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_PrecLand_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_WPNav_ArduCopter ${objs_AC_WPNav_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AC_WPNav_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AC_WPNav_ArduCopter PRIVATE ${objs_AC_WPNav_ArduCopter_INCLUDES})

set_target_properties(objs_AC_WPNav_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_WPNav_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_WPNav_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Motors_ArduCopter ${objs_AP_Motors_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Motors_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Motors_ArduCopter PRIVATE ${objs_AP_Motors_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Motors_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Motors_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Motors_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Avoidance_ArduCopter ${objs_AP_Avoidance_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Avoidance_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Avoidance_ArduCopter PRIVATE ${objs_AP_Avoidance_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Avoidance_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Avoidance_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Avoidance_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Avoidance_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_AutoTune_ArduCopter ${objs_AC_AutoTune_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AC_AutoTune_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AC_AutoTune_ArduCopter PRIVATE ${objs_AC_AutoTune_ArduCopter_INCLUDES})

set_target_properties(objs_AC_AutoTune_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AutoTune_ArduCopter mavlink) #clean
target_link_libraries(objs_AC_AutoTune_ArduCopter dronecan) #clean
target_link_libraries(objs_AC_AutoTune_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_HAL_SITL_ArduCopter ${objs_AP_HAL_SITL_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_SITL_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_HAL_SITL_ArduCopter PRIVATE ${objs_AP_HAL_SITL_ArduCopter_INCLUDES})

set_target_properties(objs_AP_HAL_SITL_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_SITL_ArduCopter ${objs_SITL_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_SITL_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_SITL_ArduCopter PRIVATE ${objs_SITL_ArduCopter_INCLUDES})

set_target_properties(objs_SITL_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_ArduCopter mavlink) #clean
target_link_libraries(objs_SITL_ArduCopter dronecan) #clean
target_link_libraries(objs_SITL_ArduCopter littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Scripting_ArduCopter ${objs_AP_Scripting_ArduCopter_SOURCES}) #2
target_compile_definitions(objs_AP_Scripting_ArduCopter PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Scripting_ArduCopter PRIVATE ${objs_AP_Scripting_ArduCopter_INCLUDES})

set_target_properties(objs_AP_Scripting_ArduCopter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_ArduCopter mavlink) #clean
target_link_libraries(objs_AP_Scripting_ArduCopter dronecan) #clean
target_link_libraries(objs_AP_Scripting_ArduCopter littlefs) #clean

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


target_link_libraries(ArduCopter_libs objs_AP_AccelCal_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_ADC_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_AHRS_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Airspeed_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Baro_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_BattMonitor_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_BoardConfig_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Camera_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Common_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Compass_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Declination_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_GPS_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_GSOF_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_HAL_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_HAL_Empty_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_DDS_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_InertialSensor_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Math_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Mission_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_DAL_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_NavEKF_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_NavEKF2_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_NavEKF3_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Notify_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_OpticalFlow_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Param_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Rally_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_LightWareSerial_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RangeFinder_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Scheduler_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_SerialManager_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Terrain_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Vehicle_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_InternalError_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Logger_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_Filter_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_GCS_MAVLink_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_RC_Channel_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_SRV_Channel_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_StorageManager_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Tuning_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RPM_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RSSI_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Mount_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Module_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Button_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_ICEngine_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Networking_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Frsky_Telem_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_IBus_Telem_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_FlashStorage_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Relay_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_ServoRelayEvents_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Volz_Protocol_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_SBusOut_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_IOMCU_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Parachute_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RAMTRON_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RCProtocol_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Radio_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_TempCalibration_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_VisualOdom_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_BLHeli_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_ROMFS_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Proximity_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Gripper_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RTC_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_Sprayer_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_Fence_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_Avoidance_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_LandingGear_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RobotisServo_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_NMEA_Output_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_OSD_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Filesystem_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_ADSB_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_PID_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_SerialLED_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_EFI_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Hott_Telem_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_ESC_Telem_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Servo_Telem_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Stats_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_GyroFFT_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RCTelemetry_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Generator_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_MSP_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_OLC_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_WheelEncoder_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_ExternalAHRS_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_VideoTX_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_FETtecOneWire_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_TemperatureSensor_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Torqeedo_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_CustomRotations_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_AIS_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_OpenDroneID_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_CheckFirmware_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_ExternalControl_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_JSON_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Beacon_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Arming_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_RCMapper_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_MultiHeap_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Follow_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_CANManager_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_KDECAN_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_PiccoloCAN_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_AttitudeControl_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_InputManager_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_PrecLand_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_Autorotation_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_WPNav_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_IRLock_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Motors_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Avoidance_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_AdvancedFailsafe_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_SmartRTL_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Winch_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_LTM_Telem_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Devo_Telem_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_AutoTune_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_SurfaceDistance_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_DroneCAN_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_HAL_SITL_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_CSVReader_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_SITL_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AP_Scripting_ArduCopter) #stripped
target_link_libraries(ArduCopter_libs objs_AC_CustomControl_ArduCopter) #stripped

#------------------------------------------------

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

add_executable(bin_arducopter ${bin_arducopter_SOURCES})
target_compile_definitions(bin_arducopter PRIVATE -DFRAME_CONFIG=MULTICOPTER_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(bin_arducopter PRIVATE ${bin_arducopter_INCLUDES})


target_link_libraries(bin_arducopter ArduCopter_libs) #clean

#------------------------------------------------

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

add_executable(bin_arducopter-heli ${bin_arducopter-heli_SOURCES})
target_compile_definitions(bin_arducopter-heli PRIVATE -DFRAME_CONFIG=HELI_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(bin_arducopter-heli PRIVATE ${bin_arducopter-heli_INCLUDES})


target_link_libraries(bin_arducopter-heli ArduCopter_libs) #clean

#------------------------------------------------

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

add_library(objs_AP_AHRS_ArduPlane ${objs_AP_AHRS_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_AHRS_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_AHRS_ArduPlane PRIVATE ${objs_AP_AHRS_ArduPlane_INCLUDES})

set_target_properties(objs_AP_AHRS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_AHRS_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_AHRS_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Airspeed_ArduPlane ${objs_AP_Airspeed_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Airspeed_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Airspeed_ArduPlane PRIVATE ${objs_AP_Airspeed_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Airspeed_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Airspeed_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Airspeed_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Baro_ArduPlane ${objs_AP_Baro_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Baro_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Baro_ArduPlane PRIVATE ${objs_AP_Baro_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Baro_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Baro_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Baro_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BattMonitor_ArduPlane ${objs_AP_BattMonitor_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_BattMonitor_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_BattMonitor_ArduPlane PRIVATE ${objs_AP_BattMonitor_ArduPlane_INCLUDES})

set_target_properties(objs_AP_BattMonitor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BoardConfig_ArduPlane ${objs_AP_BoardConfig_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_BoardConfig_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_BoardConfig_ArduPlane PRIVATE ${objs_AP_BoardConfig_ArduPlane_INCLUDES})

set_target_properties(objs_AP_BoardConfig_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_HAL_ArduPlane ${objs_AP_HAL_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_HAL_ArduPlane PRIVATE ${objs_AP_HAL_ArduPlane_INCLUDES})

set_target_properties(objs_AP_HAL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_HAL_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_HAL_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_InertialSensor_ArduPlane ${objs_AP_InertialSensor_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_InertialSensor_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_InertialSensor_ArduPlane PRIVATE ${objs_AP_InertialSensor_ArduPlane_INCLUDES})

set_target_properties(objs_AP_InertialSensor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Math_ArduPlane ${objs_AP_Math_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Math_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Math_ArduPlane PRIVATE ${objs_AP_Math_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Math_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Math_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Math_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Mission_ArduPlane ${objs_AP_Mission_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Mission_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Mission_ArduPlane PRIVATE ${objs_AP_Mission_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Mission_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Mission_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Mission_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_DAL_ArduPlane ${objs_AP_DAL_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_DAL_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_DAL_ArduPlane PRIVATE ${objs_AP_DAL_ArduPlane_INCLUDES})

set_target_properties(objs_AP_DAL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_DAL_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_DAL_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_NavEKF2_ArduPlane ${objs_AP_NavEKF2_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF2_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_NavEKF2_ArduPlane PRIVATE ${objs_AP_NavEKF2_ArduPlane_INCLUDES})

set_target_properties(objs_AP_NavEKF2_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_NavEKF3_ArduPlane ${objs_AP_NavEKF3_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF3_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_NavEKF3_ArduPlane PRIVATE ${objs_AP_NavEKF3_ArduPlane_INCLUDES})

set_target_properties(objs_AP_NavEKF3_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Rally_ArduPlane ${objs_AP_Rally_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Rally_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Rally_ArduPlane PRIVATE ${objs_AP_Rally_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Rally_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Rally_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Rally_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Scheduler_ArduPlane ${objs_AP_Scheduler_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Scheduler_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Scheduler_ArduPlane PRIVATE ${objs_AP_Scheduler_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Scheduler_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Scheduler_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Scheduler_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Terrain_ArduPlane ${objs_AP_Terrain_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Terrain_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Terrain_ArduPlane PRIVATE ${objs_AP_Terrain_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Terrain_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Terrain_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Terrain_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Vehicle_ArduPlane ${objs_AP_Vehicle_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Vehicle_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Vehicle_ArduPlane PRIVATE ${objs_AP_Vehicle_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Vehicle_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Vehicle_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Vehicle_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Logger_ArduPlane ${objs_AP_Logger_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Logger_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Logger_ArduPlane PRIVATE ${objs_AP_Logger_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Logger_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Logger_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Logger_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_Filter_ArduPlane ${objs_Filter_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_Filter_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_Filter_ArduPlane PRIVATE ${objs_Filter_ArduPlane_INCLUDES})

set_target_properties(objs_Filter_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_ArduPlane mavlink) #clean
target_link_libraries(objs_Filter_ArduPlane dronecan) #clean
target_link_libraries(objs_Filter_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_GCS_MAVLink_ArduPlane ${objs_GCS_MAVLink_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_GCS_MAVLink_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_GCS_MAVLink_ArduPlane PRIVATE ${objs_GCS_MAVLink_ArduPlane_INCLUDES})

set_target_properties(objs_GCS_MAVLink_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_ArduPlane mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_ArduPlane dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_RC_Channel_ArduPlane ${objs_RC_Channel_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_RC_Channel_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_RC_Channel_ArduPlane PRIVATE ${objs_RC_Channel_ArduPlane_INCLUDES})

set_target_properties(objs_RC_Channel_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_ArduPlane mavlink) #clean
target_link_libraries(objs_RC_Channel_ArduPlane dronecan) #clean
target_link_libraries(objs_RC_Channel_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_StorageManager_ArduPlane ${objs_StorageManager_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_StorageManager_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_StorageManager_ArduPlane PRIVATE ${objs_StorageManager_ArduPlane_INCLUDES})

set_target_properties(objs_StorageManager_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_ArduPlane mavlink) #clean
target_link_libraries(objs_StorageManager_ArduPlane dronecan) #clean
target_link_libraries(objs_StorageManager_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Frsky_Telem_ArduPlane ${objs_AP_Frsky_Telem_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Frsky_Telem_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Frsky_Telem_ArduPlane PRIVATE ${objs_AP_Frsky_Telem_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Frsky_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_IBus_Telem_ArduPlane ${objs_AP_IBus_Telem_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_IBus_Telem_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_IBus_Telem_ArduPlane PRIVATE ${objs_AP_IBus_Telem_ArduPlane_INCLUDES})

set_target_properties(objs_AP_IBus_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Relay_ArduPlane ${objs_AP_Relay_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Relay_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Relay_ArduPlane PRIVATE ${objs_AP_Relay_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Relay_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Relay_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Relay_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_RCProtocol_ArduPlane ${objs_AP_RCProtocol_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_RCProtocol_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_RCProtocol_ArduPlane PRIVATE ${objs_AP_RCProtocol_ArduPlane_INCLUDES})

set_target_properties(objs_AP_RCProtocol_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BLHeli_ArduPlane ${objs_AP_BLHeli_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_BLHeli_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_BLHeli_ArduPlane PRIVATE ${objs_AP_BLHeli_ArduPlane_INCLUDES})

set_target_properties(objs_AP_BLHeli_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_BLHeli_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_BLHeli_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Proximity_ArduPlane ${objs_AP_Proximity_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Proximity_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Proximity_ArduPlane PRIVATE ${objs_AP_Proximity_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Proximity_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Proximity_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Proximity_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_Fence_ArduPlane ${objs_AC_Fence_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AC_Fence_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AC_Fence_ArduPlane PRIVATE ${objs_AC_Fence_ArduPlane_INCLUDES})

set_target_properties(objs_AC_Fence_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_Fence_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_Fence_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_Avoidance_ArduPlane ${objs_AC_Avoidance_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AC_Avoidance_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AC_Avoidance_ArduPlane PRIVATE ${objs_AC_Avoidance_ArduPlane_INCLUDES})

set_target_properties(objs_AC_Avoidance_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_Avoidance_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_Avoidance_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_LandingGear_ArduPlane ${objs_AP_LandingGear_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_LandingGear_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_LandingGear_ArduPlane PRIVATE ${objs_AP_LandingGear_ArduPlane_INCLUDES})

set_target_properties(objs_AP_LandingGear_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_LandingGear_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_LandingGear_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_OSD_ArduPlane ${objs_AP_OSD_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_OSD_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_OSD_ArduPlane PRIVATE ${objs_AP_OSD_ArduPlane_INCLUDES})

set_target_properties(objs_AP_OSD_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_OSD_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_OSD_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Filesystem_ArduPlane ${objs_AP_Filesystem_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Filesystem_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Filesystem_ArduPlane PRIVATE ${objs_AP_Filesystem_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Filesystem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Filesystem_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Filesystem_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ADSB_ArduPlane ${objs_AP_ADSB_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_ADSB_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_ADSB_ArduPlane PRIVATE ${objs_AP_ADSB_ArduPlane_INCLUDES})

set_target_properties(objs_AP_ADSB_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_ADSB_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_ADSB_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ESC_Telem_ArduPlane ${objs_AP_ESC_Telem_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_ESC_Telem_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_ESC_Telem_ArduPlane PRIVATE ${objs_AP_ESC_Telem_ArduPlane_INCLUDES})

set_target_properties(objs_AP_ESC_Telem_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_GyroFFT_ArduPlane ${objs_AP_GyroFFT_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_GyroFFT_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_GyroFFT_ArduPlane PRIVATE ${objs_AP_GyroFFT_ArduPlane_INCLUDES})

set_target_properties(objs_AP_GyroFFT_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_RCTelemetry_ArduPlane ${objs_AP_RCTelemetry_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_RCTelemetry_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_RCTelemetry_ArduPlane PRIVATE ${objs_AP_RCTelemetry_ArduPlane_INCLUDES})

set_target_properties(objs_AP_RCTelemetry_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ExternalAHRS_ArduPlane ${objs_AP_ExternalAHRS_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_ExternalAHRS_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_ExternalAHRS_ArduPlane PRIVATE ${objs_AP_ExternalAHRS_ArduPlane_INCLUDES})

set_target_properties(objs_AP_ExternalAHRS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_TemperatureSensor_ArduPlane ${objs_AP_TemperatureSensor_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_TemperatureSensor_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_TemperatureSensor_ArduPlane PRIVATE ${objs_AP_TemperatureSensor_ArduPlane_INCLUDES})

set_target_properties(objs_AP_TemperatureSensor_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_AIS_ArduPlane ${objs_AP_AIS_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_AIS_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_AIS_ArduPlane PRIVATE ${objs_AP_AIS_ArduPlane_INCLUDES})

set_target_properties(objs_AP_AIS_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_AIS_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_AIS_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Arming_ArduPlane ${objs_AP_Arming_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Arming_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Arming_ArduPlane PRIVATE ${objs_AP_Arming_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Arming_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Arming_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Arming_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Follow_ArduPlane ${objs_AP_Follow_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Follow_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Follow_ArduPlane PRIVATE ${objs_AP_Follow_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Follow_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Follow_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Follow_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Avoidance_ArduPlane ${objs_AP_Avoidance_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Avoidance_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Avoidance_ArduPlane PRIVATE ${objs_AP_Avoidance_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Avoidance_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Avoidance_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Avoidance_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Avoidance_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_WPNav_ArduPlane ${objs_AC_WPNav_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AC_WPNav_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AC_WPNav_ArduPlane PRIVATE ${objs_AC_WPNav_ArduPlane_INCLUDES})

set_target_properties(objs_AC_WPNav_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_WPNav_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_WPNav_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_AttitudeControl_ArduPlane ${objs_AC_AttitudeControl_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AC_AttitudeControl_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AC_AttitudeControl_ArduPlane PRIVATE ${objs_AC_AttitudeControl_ArduPlane_INCLUDES})

set_target_properties(objs_AC_AttitudeControl_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Motors_ArduPlane ${objs_AP_Motors_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Motors_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Motors_ArduPlane PRIVATE ${objs_AP_Motors_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Motors_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Motors_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Motors_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Landing_ArduPlane ${objs_AP_Landing_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Landing_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Landing_ArduPlane PRIVATE ${objs_AP_Landing_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Landing_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Landing_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Landing_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Landing_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_AutoTune_ArduPlane ${objs_AC_AutoTune_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AC_AutoTune_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AC_AutoTune_ArduPlane PRIVATE ${objs_AC_AutoTune_ArduPlane_INCLUDES})

set_target_properties(objs_AC_AutoTune_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AutoTune_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_AutoTune_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_AutoTune_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_PrecLand_ArduPlane ${objs_AC_PrecLand_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AC_PrecLand_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AC_PrecLand_ArduPlane PRIVATE ${objs_AC_PrecLand_ArduPlane_INCLUDES})

set_target_properties(objs_AC_PrecLand_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_PrecLand_ArduPlane mavlink) #clean
target_link_libraries(objs_AC_PrecLand_ArduPlane dronecan) #clean
target_link_libraries(objs_AC_PrecLand_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_HAL_SITL_ArduPlane ${objs_AP_HAL_SITL_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_SITL_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_HAL_SITL_ArduPlane PRIVATE ${objs_AP_HAL_SITL_ArduPlane_INCLUDES})

set_target_properties(objs_AP_HAL_SITL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_SITL_ArduPlane ${objs_SITL_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_SITL_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_SITL_ArduPlane PRIVATE ${objs_SITL_ArduPlane_INCLUDES})

set_target_properties(objs_SITL_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_ArduPlane mavlink) #clean
target_link_libraries(objs_SITL_ArduPlane dronecan) #clean
target_link_libraries(objs_SITL_ArduPlane littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Scripting_ArduPlane ${objs_AP_Scripting_ArduPlane_SOURCES}) #2
target_compile_definitions(objs_AP_Scripting_ArduPlane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(objs_AP_Scripting_ArduPlane PRIVATE ${objs_AP_Scripting_ArduPlane_INCLUDES})

set_target_properties(objs_AP_Scripting_ArduPlane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_ArduPlane mavlink) #clean
target_link_libraries(objs_AP_Scripting_ArduPlane dronecan) #clean
target_link_libraries(objs_AP_Scripting_ArduPlane littlefs) #clean

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


target_link_libraries(ArduPlane_libs objs_AP_AccelCal_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_ADC_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_AHRS_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Airspeed_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Baro_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_BattMonitor_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_BoardConfig_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Camera_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Common_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Compass_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Declination_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_GPS_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_GSOF_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_HAL_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_HAL_Empty_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_DDS_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_InertialSensor_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Math_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Mission_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_DAL_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_NavEKF_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_NavEKF2_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_NavEKF3_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Notify_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_OpticalFlow_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Param_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Rally_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_LightWareSerial_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RangeFinder_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Scheduler_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_SerialManager_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Terrain_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Vehicle_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_InternalError_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Logger_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_Filter_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_GCS_MAVLink_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_RC_Channel_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_SRV_Channel_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_StorageManager_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Tuning_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RPM_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RSSI_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Mount_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Module_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Button_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_ICEngine_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Networking_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Frsky_Telem_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_IBus_Telem_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_FlashStorage_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Relay_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_ServoRelayEvents_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Volz_Protocol_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_SBusOut_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_IOMCU_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Parachute_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RAMTRON_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RCProtocol_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Radio_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_TempCalibration_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_VisualOdom_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_BLHeli_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_ROMFS_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Proximity_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Gripper_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RTC_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_Sprayer_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_Fence_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_Avoidance_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_LandingGear_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RobotisServo_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_NMEA_Output_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_OSD_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Filesystem_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_ADSB_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_PID_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_SerialLED_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_EFI_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Hott_Telem_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_ESC_Telem_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Servo_Telem_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Stats_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_GyroFFT_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RCTelemetry_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Generator_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_MSP_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_OLC_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_WheelEncoder_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_ExternalAHRS_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_VideoTX_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_FETtecOneWire_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_TemperatureSensor_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Torqeedo_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_CustomRotations_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_AIS_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_OpenDroneID_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_CheckFirmware_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_ExternalControl_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_JSON_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Beacon_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Arming_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_RCMapper_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_MultiHeap_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Follow_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_CANManager_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_KDECAN_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_PiccoloCAN_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_APM_Control_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_AdvancedFailsafe_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Avoidance_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_L1_Control_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Navigation_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_TECS_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_InertialNav_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_WPNav_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_AttitudeControl_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Motors_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Landing_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_PID_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Soaring_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_LTM_Telem_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Devo_Telem_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_AutoTune_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_PrecLand_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_IRLock_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Quicktune_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_DroneCAN_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_HAL_SITL_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_CSVReader_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_SITL_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AP_Scripting_ArduPlane) #stripped
target_link_libraries(ArduPlane_libs objs_AC_CustomControl_ArduPlane) #stripped

#------------------------------------------------

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

add_executable(bin_arduplane ${bin_arduplane_SOURCES})
target_compile_definitions(bin_arduplane PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_include_directories(bin_arduplane PRIVATE ${bin_arduplane_INCLUDES})


target_link_libraries(bin_arduplane ArduPlane_libs) #clean

#------------------------------------------------

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

add_library(objs_AP_AHRS_ArduSub ${objs_AP_AHRS_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_AHRS_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_AHRS_ArduSub PRIVATE ${objs_AP_AHRS_ArduSub_INCLUDES})

set_target_properties(objs_AP_AHRS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_ArduSub mavlink) #clean
target_link_libraries(objs_AP_AHRS_ArduSub dronecan) #clean
target_link_libraries(objs_AP_AHRS_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Airspeed_ArduSub ${objs_AP_Airspeed_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Airspeed_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Airspeed_ArduSub PRIVATE ${objs_AP_Airspeed_ArduSub_INCLUDES})

set_target_properties(objs_AP_Airspeed_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Airspeed_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Airspeed_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Baro_ArduSub ${objs_AP_Baro_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Baro_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Baro_ArduSub PRIVATE ${objs_AP_Baro_ArduSub_INCLUDES})

set_target_properties(objs_AP_Baro_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Baro_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Baro_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BattMonitor_ArduSub ${objs_AP_BattMonitor_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_BattMonitor_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_BattMonitor_ArduSub PRIVATE ${objs_AP_BattMonitor_ArduSub_INCLUDES})

set_target_properties(objs_AP_BattMonitor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_ArduSub mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_ArduSub dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BoardConfig_ArduSub ${objs_AP_BoardConfig_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_BoardConfig_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_BoardConfig_ArduSub PRIVATE ${objs_AP_BoardConfig_ArduSub_INCLUDES})

set_target_properties(objs_AP_BoardConfig_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_ArduSub mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_ArduSub dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_HAL_ArduSub ${objs_AP_HAL_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_HAL_ArduSub PRIVATE ${objs_AP_HAL_ArduSub_INCLUDES})

set_target_properties(objs_AP_HAL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_ArduSub mavlink) #clean
target_link_libraries(objs_AP_HAL_ArduSub dronecan) #clean
target_link_libraries(objs_AP_HAL_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_InertialSensor_ArduSub ${objs_AP_InertialSensor_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_InertialSensor_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_InertialSensor_ArduSub PRIVATE ${objs_AP_InertialSensor_ArduSub_INCLUDES})

set_target_properties(objs_AP_InertialSensor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_ArduSub mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_ArduSub dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Math_ArduSub ${objs_AP_Math_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Math_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Math_ArduSub PRIVATE ${objs_AP_Math_ArduSub_INCLUDES})

set_target_properties(objs_AP_Math_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Math_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Math_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Mission_ArduSub ${objs_AP_Mission_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Mission_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Mission_ArduSub PRIVATE ${objs_AP_Mission_ArduSub_INCLUDES})

set_target_properties(objs_AP_Mission_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Mission_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Mission_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_DAL_ArduSub ${objs_AP_DAL_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_DAL_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_DAL_ArduSub PRIVATE ${objs_AP_DAL_ArduSub_INCLUDES})

set_target_properties(objs_AP_DAL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_ArduSub mavlink) #clean
target_link_libraries(objs_AP_DAL_ArduSub dronecan) #clean
target_link_libraries(objs_AP_DAL_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_NavEKF2_ArduSub ${objs_AP_NavEKF2_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF2_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_NavEKF2_ArduSub PRIVATE ${objs_AP_NavEKF2_ArduSub_INCLUDES})

set_target_properties(objs_AP_NavEKF2_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_ArduSub mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_ArduSub dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_NavEKF3_ArduSub ${objs_AP_NavEKF3_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF3_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_NavEKF3_ArduSub PRIVATE ${objs_AP_NavEKF3_ArduSub_INCLUDES})

set_target_properties(objs_AP_NavEKF3_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_ArduSub mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_ArduSub dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Rally_ArduSub ${objs_AP_Rally_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Rally_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Rally_ArduSub PRIVATE ${objs_AP_Rally_ArduSub_INCLUDES})

set_target_properties(objs_AP_Rally_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Rally_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Rally_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Scheduler_ArduSub ${objs_AP_Scheduler_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Scheduler_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Scheduler_ArduSub PRIVATE ${objs_AP_Scheduler_ArduSub_INCLUDES})

set_target_properties(objs_AP_Scheduler_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Scheduler_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Scheduler_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Terrain_ArduSub ${objs_AP_Terrain_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Terrain_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Terrain_ArduSub PRIVATE ${objs_AP_Terrain_ArduSub_INCLUDES})

set_target_properties(objs_AP_Terrain_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Terrain_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Terrain_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Vehicle_ArduSub ${objs_AP_Vehicle_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Vehicle_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Vehicle_ArduSub PRIVATE ${objs_AP_Vehicle_ArduSub_INCLUDES})

set_target_properties(objs_AP_Vehicle_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Vehicle_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Vehicle_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Logger_ArduSub ${objs_AP_Logger_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Logger_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Logger_ArduSub PRIVATE ${objs_AP_Logger_ArduSub_INCLUDES})

set_target_properties(objs_AP_Logger_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Logger_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Logger_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_Filter_ArduSub ${objs_Filter_ArduSub_SOURCES}) #2
target_compile_definitions(objs_Filter_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_Filter_ArduSub PRIVATE ${objs_Filter_ArduSub_INCLUDES})

set_target_properties(objs_Filter_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_ArduSub mavlink) #clean
target_link_libraries(objs_Filter_ArduSub dronecan) #clean
target_link_libraries(objs_Filter_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_GCS_MAVLink_ArduSub ${objs_GCS_MAVLink_ArduSub_SOURCES}) #2
target_compile_definitions(objs_GCS_MAVLink_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_GCS_MAVLink_ArduSub PRIVATE ${objs_GCS_MAVLink_ArduSub_INCLUDES})

set_target_properties(objs_GCS_MAVLink_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_ArduSub mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_ArduSub dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_RC_Channel_ArduSub ${objs_RC_Channel_ArduSub_SOURCES}) #2
target_compile_definitions(objs_RC_Channel_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_RC_Channel_ArduSub PRIVATE ${objs_RC_Channel_ArduSub_INCLUDES})

set_target_properties(objs_RC_Channel_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_ArduSub mavlink) #clean
target_link_libraries(objs_RC_Channel_ArduSub dronecan) #clean
target_link_libraries(objs_RC_Channel_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_StorageManager_ArduSub ${objs_StorageManager_ArduSub_SOURCES}) #2
target_compile_definitions(objs_StorageManager_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_StorageManager_ArduSub PRIVATE ${objs_StorageManager_ArduSub_INCLUDES})

set_target_properties(objs_StorageManager_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_ArduSub mavlink) #clean
target_link_libraries(objs_StorageManager_ArduSub dronecan) #clean
target_link_libraries(objs_StorageManager_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Frsky_Telem_ArduSub ${objs_AP_Frsky_Telem_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Frsky_Telem_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Frsky_Telem_ArduSub PRIVATE ${objs_AP_Frsky_Telem_ArduSub_INCLUDES})

set_target_properties(objs_AP_Frsky_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_IBus_Telem_ArduSub ${objs_AP_IBus_Telem_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_IBus_Telem_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_IBus_Telem_ArduSub PRIVATE ${objs_AP_IBus_Telem_ArduSub_INCLUDES})

set_target_properties(objs_AP_IBus_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_ArduSub mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduSub dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Relay_ArduSub ${objs_AP_Relay_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Relay_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Relay_ArduSub PRIVATE ${objs_AP_Relay_ArduSub_INCLUDES})

set_target_properties(objs_AP_Relay_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Relay_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Relay_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_RCProtocol_ArduSub ${objs_AP_RCProtocol_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_RCProtocol_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_RCProtocol_ArduSub PRIVATE ${objs_AP_RCProtocol_ArduSub_INCLUDES})

set_target_properties(objs_AP_RCProtocol_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_ArduSub mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_ArduSub dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BLHeli_ArduSub ${objs_AP_BLHeli_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_BLHeli_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_BLHeli_ArduSub PRIVATE ${objs_AP_BLHeli_ArduSub_INCLUDES})

set_target_properties(objs_AP_BLHeli_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_ArduSub mavlink) #clean
target_link_libraries(objs_AP_BLHeli_ArduSub dronecan) #clean
target_link_libraries(objs_AP_BLHeli_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Proximity_ArduSub ${objs_AP_Proximity_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Proximity_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Proximity_ArduSub PRIVATE ${objs_AP_Proximity_ArduSub_INCLUDES})

set_target_properties(objs_AP_Proximity_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Proximity_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Proximity_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_Fence_ArduSub ${objs_AC_Fence_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AC_Fence_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AC_Fence_ArduSub PRIVATE ${objs_AC_Fence_ArduSub_INCLUDES})

set_target_properties(objs_AC_Fence_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_ArduSub mavlink) #clean
target_link_libraries(objs_AC_Fence_ArduSub dronecan) #clean
target_link_libraries(objs_AC_Fence_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_Avoidance_ArduSub ${objs_AC_Avoidance_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AC_Avoidance_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AC_Avoidance_ArduSub PRIVATE ${objs_AC_Avoidance_ArduSub_INCLUDES})

set_target_properties(objs_AC_Avoidance_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_ArduSub mavlink) #clean
target_link_libraries(objs_AC_Avoidance_ArduSub dronecan) #clean
target_link_libraries(objs_AC_Avoidance_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_LandingGear_ArduSub ${objs_AP_LandingGear_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_LandingGear_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_LandingGear_ArduSub PRIVATE ${objs_AP_LandingGear_ArduSub_INCLUDES})

set_target_properties(objs_AP_LandingGear_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_ArduSub mavlink) #clean
target_link_libraries(objs_AP_LandingGear_ArduSub dronecan) #clean
target_link_libraries(objs_AP_LandingGear_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_OSD_ArduSub ${objs_AP_OSD_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_OSD_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_OSD_ArduSub PRIVATE ${objs_AP_OSD_ArduSub_INCLUDES})

set_target_properties(objs_AP_OSD_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_ArduSub mavlink) #clean
target_link_libraries(objs_AP_OSD_ArduSub dronecan) #clean
target_link_libraries(objs_AP_OSD_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Filesystem_ArduSub ${objs_AP_Filesystem_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Filesystem_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Filesystem_ArduSub PRIVATE ${objs_AP_Filesystem_ArduSub_INCLUDES})

set_target_properties(objs_AP_Filesystem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Filesystem_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Filesystem_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ADSB_ArduSub ${objs_AP_ADSB_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_ADSB_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_ADSB_ArduSub PRIVATE ${objs_AP_ADSB_ArduSub_INCLUDES})

set_target_properties(objs_AP_ADSB_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_ArduSub mavlink) #clean
target_link_libraries(objs_AP_ADSB_ArduSub dronecan) #clean
target_link_libraries(objs_AP_ADSB_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ESC_Telem_ArduSub ${objs_AP_ESC_Telem_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_ESC_Telem_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_ESC_Telem_ArduSub PRIVATE ${objs_AP_ESC_Telem_ArduSub_INCLUDES})

set_target_properties(objs_AP_ESC_Telem_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_ArduSub mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduSub dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_GyroFFT_ArduSub ${objs_AP_GyroFFT_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_GyroFFT_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_GyroFFT_ArduSub PRIVATE ${objs_AP_GyroFFT_ArduSub_INCLUDES})

set_target_properties(objs_AP_GyroFFT_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_ArduSub mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_ArduSub dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_RCTelemetry_ArduSub ${objs_AP_RCTelemetry_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_RCTelemetry_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_RCTelemetry_ArduSub PRIVATE ${objs_AP_RCTelemetry_ArduSub_INCLUDES})

set_target_properties(objs_AP_RCTelemetry_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_ArduSub mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduSub dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ExternalAHRS_ArduSub ${objs_AP_ExternalAHRS_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_ExternalAHRS_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_ExternalAHRS_ArduSub PRIVATE ${objs_AP_ExternalAHRS_ArduSub_INCLUDES})

set_target_properties(objs_AP_ExternalAHRS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_ArduSub mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduSub dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_TemperatureSensor_ArduSub ${objs_AP_TemperatureSensor_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_TemperatureSensor_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_TemperatureSensor_ArduSub PRIVATE ${objs_AP_TemperatureSensor_ArduSub_INCLUDES})

set_target_properties(objs_AP_TemperatureSensor_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_ArduSub mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduSub dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_AIS_ArduSub ${objs_AP_AIS_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_AIS_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_AIS_ArduSub PRIVATE ${objs_AP_AIS_ArduSub_INCLUDES})

set_target_properties(objs_AP_AIS_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_ArduSub mavlink) #clean
target_link_libraries(objs_AP_AIS_ArduSub dronecan) #clean
target_link_libraries(objs_AP_AIS_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Arming_ArduSub ${objs_AP_Arming_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Arming_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Arming_ArduSub PRIVATE ${objs_AP_Arming_ArduSub_INCLUDES})

set_target_properties(objs_AP_Arming_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Arming_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Arming_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Follow_ArduSub ${objs_AP_Follow_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Follow_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Follow_ArduSub PRIVATE ${objs_AP_Follow_ArduSub_INCLUDES})

set_target_properties(objs_AP_Follow_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Follow_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Follow_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_AttitudeControl_ArduSub ${objs_AC_AttitudeControl_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AC_AttitudeControl_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AC_AttitudeControl_ArduSub PRIVATE ${objs_AC_AttitudeControl_ArduSub_INCLUDES})

set_target_properties(objs_AC_AttitudeControl_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_ArduSub mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduSub dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_WPNav_ArduSub ${objs_AC_WPNav_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AC_WPNav_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AC_WPNav_ArduSub PRIVATE ${objs_AC_WPNav_ArduSub_INCLUDES})

set_target_properties(objs_AC_WPNav_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_WPNav_ArduSub mavlink) #clean
target_link_libraries(objs_AC_WPNav_ArduSub dronecan) #clean
target_link_libraries(objs_AC_WPNav_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Motors_ArduSub ${objs_AP_Motors_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Motors_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Motors_ArduSub PRIVATE ${objs_AP_Motors_ArduSub_INCLUDES})

set_target_properties(objs_AP_Motors_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Motors_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Motors_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Motors_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_HAL_SITL_ArduSub ${objs_AP_HAL_SITL_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_SITL_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_HAL_SITL_ArduSub PRIVATE ${objs_AP_HAL_SITL_ArduSub_INCLUDES})

set_target_properties(objs_AP_HAL_SITL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_ArduSub mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduSub dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_SITL_ArduSub ${objs_SITL_ArduSub_SOURCES}) #2
target_compile_definitions(objs_SITL_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_SITL_ArduSub PRIVATE ${objs_SITL_ArduSub_INCLUDES})

set_target_properties(objs_SITL_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_ArduSub mavlink) #clean
target_link_libraries(objs_SITL_ArduSub dronecan) #clean
target_link_libraries(objs_SITL_ArduSub littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Scripting_ArduSub ${objs_AP_Scripting_ArduSub_SOURCES}) #2
target_compile_definitions(objs_AP_Scripting_ArduSub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(objs_AP_Scripting_ArduSub PRIVATE ${objs_AP_Scripting_ArduSub_INCLUDES})

set_target_properties(objs_AP_Scripting_ArduSub PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_ArduSub mavlink) #clean
target_link_libraries(objs_AP_Scripting_ArduSub dronecan) #clean
target_link_libraries(objs_AP_Scripting_ArduSub littlefs) #clean

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


target_link_libraries(ArduSub_libs objs_AP_AccelCal_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_ADC_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_AHRS_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Airspeed_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Baro_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_BattMonitor_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_BoardConfig_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Camera_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Common_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Compass_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Declination_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_GPS_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_GSOF_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_HAL_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_HAL_Empty_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_DDS_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_InertialSensor_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Math_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Mission_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_DAL_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_NavEKF_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_NavEKF2_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_NavEKF3_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Notify_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_OpticalFlow_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Param_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Rally_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_LightWareSerial_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RangeFinder_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Scheduler_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_SerialManager_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Terrain_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Vehicle_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_InternalError_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Logger_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_Filter_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_GCS_MAVLink_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_RC_Channel_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_SRV_Channel_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_StorageManager_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Tuning_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RPM_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RSSI_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Mount_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Module_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Button_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_ICEngine_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Networking_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Frsky_Telem_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_IBus_Telem_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_FlashStorage_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Relay_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_ServoRelayEvents_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Volz_Protocol_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_SBusOut_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_IOMCU_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Parachute_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RAMTRON_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RCProtocol_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Radio_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_TempCalibration_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_VisualOdom_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_BLHeli_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_ROMFS_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Proximity_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Gripper_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RTC_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AC_Sprayer_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AC_Fence_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AC_Avoidance_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_LandingGear_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RobotisServo_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_NMEA_Output_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_OSD_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Filesystem_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_ADSB_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AC_PID_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_SerialLED_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_EFI_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Hott_Telem_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_ESC_Telem_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Servo_Telem_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Stats_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_GyroFFT_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RCTelemetry_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Generator_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_MSP_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_OLC_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_WheelEncoder_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_ExternalAHRS_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_VideoTX_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_FETtecOneWire_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_TemperatureSensor_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Torqeedo_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_CustomRotations_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_AIS_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_OpenDroneID_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_CheckFirmware_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_ExternalControl_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_JSON_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Beacon_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Arming_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_RCMapper_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_MultiHeap_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Follow_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_CANManager_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_KDECAN_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_PiccoloCAN_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AC_AttitudeControl_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AC_WPNav_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_InertialNav_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_JSButton_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_LeakDetector_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Motors_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_DroneCAN_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_HAL_SITL_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_CSVReader_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_SITL_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AP_Scripting_ArduSub) #stripped
target_link_libraries(ArduSub_libs objs_AC_CustomControl_ArduSub) #stripped

#------------------------------------------------

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

add_executable(bin_ardusub ${bin_ardusub_SOURCES})
target_compile_definitions(bin_ardusub PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_include_directories(bin_ardusub PRIVATE ${bin_ardusub_INCLUDES})


target_link_libraries(bin_ardusub ArduSub_libs) #clean

#------------------------------------------------

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

add_library(objs_AP_AHRS_Blimp ${objs_AP_AHRS_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_AHRS_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_AHRS_Blimp PRIVATE ${objs_AP_AHRS_Blimp_INCLUDES})

set_target_properties(objs_AP_AHRS_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_Blimp mavlink) #clean
target_link_libraries(objs_AP_AHRS_Blimp dronecan) #clean
target_link_libraries(objs_AP_AHRS_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Airspeed_Blimp ${objs_AP_Airspeed_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Airspeed_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Airspeed_Blimp PRIVATE ${objs_AP_Airspeed_Blimp_INCLUDES})

set_target_properties(objs_AP_Airspeed_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_Blimp mavlink) #clean
target_link_libraries(objs_AP_Airspeed_Blimp dronecan) #clean
target_link_libraries(objs_AP_Airspeed_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Baro_Blimp ${objs_AP_Baro_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Baro_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Baro_Blimp PRIVATE ${objs_AP_Baro_Blimp_INCLUDES})

set_target_properties(objs_AP_Baro_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_Blimp mavlink) #clean
target_link_libraries(objs_AP_Baro_Blimp dronecan) #clean
target_link_libraries(objs_AP_Baro_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BattMonitor_Blimp ${objs_AP_BattMonitor_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_BattMonitor_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_BattMonitor_Blimp PRIVATE ${objs_AP_BattMonitor_Blimp_INCLUDES})

set_target_properties(objs_AP_BattMonitor_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_Blimp mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_Blimp dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BoardConfig_Blimp ${objs_AP_BoardConfig_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_BoardConfig_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_BoardConfig_Blimp PRIVATE ${objs_AP_BoardConfig_Blimp_INCLUDES})

set_target_properties(objs_AP_BoardConfig_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_Blimp mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_Blimp dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_HAL_Blimp ${objs_AP_HAL_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_HAL_Blimp PRIVATE ${objs_AP_HAL_Blimp_INCLUDES})

set_target_properties(objs_AP_HAL_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_Blimp mavlink) #clean
target_link_libraries(objs_AP_HAL_Blimp dronecan) #clean
target_link_libraries(objs_AP_HAL_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_InertialSensor_Blimp ${objs_AP_InertialSensor_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_InertialSensor_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_InertialSensor_Blimp PRIVATE ${objs_AP_InertialSensor_Blimp_INCLUDES})

set_target_properties(objs_AP_InertialSensor_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_Blimp mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_Blimp dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Math_Blimp ${objs_AP_Math_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Math_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Math_Blimp PRIVATE ${objs_AP_Math_Blimp_INCLUDES})

set_target_properties(objs_AP_Math_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_Blimp mavlink) #clean
target_link_libraries(objs_AP_Math_Blimp dronecan) #clean
target_link_libraries(objs_AP_Math_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Mission_Blimp ${objs_AP_Mission_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Mission_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Mission_Blimp PRIVATE ${objs_AP_Mission_Blimp_INCLUDES})

set_target_properties(objs_AP_Mission_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_Blimp mavlink) #clean
target_link_libraries(objs_AP_Mission_Blimp dronecan) #clean
target_link_libraries(objs_AP_Mission_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_DAL_Blimp ${objs_AP_DAL_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_DAL_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_DAL_Blimp PRIVATE ${objs_AP_DAL_Blimp_INCLUDES})

set_target_properties(objs_AP_DAL_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_Blimp mavlink) #clean
target_link_libraries(objs_AP_DAL_Blimp dronecan) #clean
target_link_libraries(objs_AP_DAL_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_NavEKF2_Blimp ${objs_AP_NavEKF2_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF2_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_NavEKF2_Blimp PRIVATE ${objs_AP_NavEKF2_Blimp_INCLUDES})

set_target_properties(objs_AP_NavEKF2_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_Blimp mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_Blimp dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_NavEKF3_Blimp ${objs_AP_NavEKF3_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF3_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_NavEKF3_Blimp PRIVATE ${objs_AP_NavEKF3_Blimp_INCLUDES})

set_target_properties(objs_AP_NavEKF3_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_Blimp mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_Blimp dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Rally_Blimp ${objs_AP_Rally_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Rally_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Rally_Blimp PRIVATE ${objs_AP_Rally_Blimp_INCLUDES})

set_target_properties(objs_AP_Rally_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_Blimp mavlink) #clean
target_link_libraries(objs_AP_Rally_Blimp dronecan) #clean
target_link_libraries(objs_AP_Rally_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Scheduler_Blimp ${objs_AP_Scheduler_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Scheduler_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Scheduler_Blimp PRIVATE ${objs_AP_Scheduler_Blimp_INCLUDES})

set_target_properties(objs_AP_Scheduler_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_Blimp mavlink) #clean
target_link_libraries(objs_AP_Scheduler_Blimp dronecan) #clean
target_link_libraries(objs_AP_Scheduler_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Terrain_Blimp ${objs_AP_Terrain_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Terrain_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Terrain_Blimp PRIVATE ${objs_AP_Terrain_Blimp_INCLUDES})

set_target_properties(objs_AP_Terrain_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_Blimp mavlink) #clean
target_link_libraries(objs_AP_Terrain_Blimp dronecan) #clean
target_link_libraries(objs_AP_Terrain_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Vehicle_Blimp ${objs_AP_Vehicle_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Vehicle_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Vehicle_Blimp PRIVATE ${objs_AP_Vehicle_Blimp_INCLUDES})

set_target_properties(objs_AP_Vehicle_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_Blimp mavlink) #clean
target_link_libraries(objs_AP_Vehicle_Blimp dronecan) #clean
target_link_libraries(objs_AP_Vehicle_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Logger_Blimp ${objs_AP_Logger_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Logger_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Logger_Blimp PRIVATE ${objs_AP_Logger_Blimp_INCLUDES})

set_target_properties(objs_AP_Logger_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_Blimp mavlink) #clean
target_link_libraries(objs_AP_Logger_Blimp dronecan) #clean
target_link_libraries(objs_AP_Logger_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_Filter_Blimp ${objs_Filter_Blimp_SOURCES}) #2
target_compile_definitions(objs_Filter_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_Filter_Blimp PRIVATE ${objs_Filter_Blimp_INCLUDES})

set_target_properties(objs_Filter_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_Blimp mavlink) #clean
target_link_libraries(objs_Filter_Blimp dronecan) #clean
target_link_libraries(objs_Filter_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_GCS_MAVLink_Blimp ${objs_GCS_MAVLink_Blimp_SOURCES}) #2
target_compile_definitions(objs_GCS_MAVLink_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_GCS_MAVLink_Blimp PRIVATE ${objs_GCS_MAVLink_Blimp_INCLUDES})

set_target_properties(objs_GCS_MAVLink_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_Blimp mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_Blimp dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_RC_Channel_Blimp ${objs_RC_Channel_Blimp_SOURCES}) #2
target_compile_definitions(objs_RC_Channel_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_RC_Channel_Blimp PRIVATE ${objs_RC_Channel_Blimp_INCLUDES})

set_target_properties(objs_RC_Channel_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_Blimp mavlink) #clean
target_link_libraries(objs_RC_Channel_Blimp dronecan) #clean
target_link_libraries(objs_RC_Channel_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_StorageManager_Blimp ${objs_StorageManager_Blimp_SOURCES}) #2
target_compile_definitions(objs_StorageManager_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_StorageManager_Blimp PRIVATE ${objs_StorageManager_Blimp_INCLUDES})

set_target_properties(objs_StorageManager_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_Blimp mavlink) #clean
target_link_libraries(objs_StorageManager_Blimp dronecan) #clean
target_link_libraries(objs_StorageManager_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Frsky_Telem_Blimp ${objs_AP_Frsky_Telem_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Frsky_Telem_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Frsky_Telem_Blimp PRIVATE ${objs_AP_Frsky_Telem_Blimp_INCLUDES})

set_target_properties(objs_AP_Frsky_Telem_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_Blimp mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_Blimp dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_IBus_Telem_Blimp ${objs_AP_IBus_Telem_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_IBus_Telem_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_IBus_Telem_Blimp PRIVATE ${objs_AP_IBus_Telem_Blimp_INCLUDES})

set_target_properties(objs_AP_IBus_Telem_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_Blimp mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_Blimp dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Relay_Blimp ${objs_AP_Relay_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Relay_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Relay_Blimp PRIVATE ${objs_AP_Relay_Blimp_INCLUDES})

set_target_properties(objs_AP_Relay_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_Blimp mavlink) #clean
target_link_libraries(objs_AP_Relay_Blimp dronecan) #clean
target_link_libraries(objs_AP_Relay_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_RCProtocol_Blimp ${objs_AP_RCProtocol_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_RCProtocol_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_RCProtocol_Blimp PRIVATE ${objs_AP_RCProtocol_Blimp_INCLUDES})

set_target_properties(objs_AP_RCProtocol_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_Blimp mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_Blimp dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_BLHeli_Blimp ${objs_AP_BLHeli_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_BLHeli_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_BLHeli_Blimp PRIVATE ${objs_AP_BLHeli_Blimp_INCLUDES})

set_target_properties(objs_AP_BLHeli_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_Blimp mavlink) #clean
target_link_libraries(objs_AP_BLHeli_Blimp dronecan) #clean
target_link_libraries(objs_AP_BLHeli_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Proximity_Blimp ${objs_AP_Proximity_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Proximity_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Proximity_Blimp PRIVATE ${objs_AP_Proximity_Blimp_INCLUDES})

set_target_properties(objs_AP_Proximity_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_Blimp mavlink) #clean
target_link_libraries(objs_AP_Proximity_Blimp dronecan) #clean
target_link_libraries(objs_AP_Proximity_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_Fence_Blimp ${objs_AC_Fence_Blimp_SOURCES}) #2
target_compile_definitions(objs_AC_Fence_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AC_Fence_Blimp PRIVATE ${objs_AC_Fence_Blimp_INCLUDES})

set_target_properties(objs_AC_Fence_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_Blimp mavlink) #clean
target_link_libraries(objs_AC_Fence_Blimp dronecan) #clean
target_link_libraries(objs_AC_Fence_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_Avoidance_Blimp ${objs_AC_Avoidance_Blimp_SOURCES}) #2
target_compile_definitions(objs_AC_Avoidance_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AC_Avoidance_Blimp PRIVATE ${objs_AC_Avoidance_Blimp_INCLUDES})

set_target_properties(objs_AC_Avoidance_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_Blimp mavlink) #clean
target_link_libraries(objs_AC_Avoidance_Blimp dronecan) #clean
target_link_libraries(objs_AC_Avoidance_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_LandingGear_Blimp ${objs_AP_LandingGear_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_LandingGear_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_LandingGear_Blimp PRIVATE ${objs_AP_LandingGear_Blimp_INCLUDES})

set_target_properties(objs_AP_LandingGear_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_Blimp mavlink) #clean
target_link_libraries(objs_AP_LandingGear_Blimp dronecan) #clean
target_link_libraries(objs_AP_LandingGear_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_OSD_Blimp ${objs_AP_OSD_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_OSD_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_OSD_Blimp PRIVATE ${objs_AP_OSD_Blimp_INCLUDES})

set_target_properties(objs_AP_OSD_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_Blimp mavlink) #clean
target_link_libraries(objs_AP_OSD_Blimp dronecan) #clean
target_link_libraries(objs_AP_OSD_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Filesystem_Blimp ${objs_AP_Filesystem_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Filesystem_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Filesystem_Blimp PRIVATE ${objs_AP_Filesystem_Blimp_INCLUDES})

set_target_properties(objs_AP_Filesystem_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_Blimp mavlink) #clean
target_link_libraries(objs_AP_Filesystem_Blimp dronecan) #clean
target_link_libraries(objs_AP_Filesystem_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ADSB_Blimp ${objs_AP_ADSB_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_ADSB_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_ADSB_Blimp PRIVATE ${objs_AP_ADSB_Blimp_INCLUDES})

set_target_properties(objs_AP_ADSB_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_Blimp mavlink) #clean
target_link_libraries(objs_AP_ADSB_Blimp dronecan) #clean
target_link_libraries(objs_AP_ADSB_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ESC_Telem_Blimp ${objs_AP_ESC_Telem_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_ESC_Telem_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_ESC_Telem_Blimp PRIVATE ${objs_AP_ESC_Telem_Blimp_INCLUDES})

set_target_properties(objs_AP_ESC_Telem_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_Blimp mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_Blimp dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_GyroFFT_Blimp ${objs_AP_GyroFFT_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_GyroFFT_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_GyroFFT_Blimp PRIVATE ${objs_AP_GyroFFT_Blimp_INCLUDES})

set_target_properties(objs_AP_GyroFFT_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_Blimp mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_Blimp dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_RCTelemetry_Blimp ${objs_AP_RCTelemetry_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_RCTelemetry_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_RCTelemetry_Blimp PRIVATE ${objs_AP_RCTelemetry_Blimp_INCLUDES})

set_target_properties(objs_AP_RCTelemetry_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_Blimp mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_Blimp dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_ExternalAHRS_Blimp ${objs_AP_ExternalAHRS_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_ExternalAHRS_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_ExternalAHRS_Blimp PRIVATE ${objs_AP_ExternalAHRS_Blimp_INCLUDES})

set_target_properties(objs_AP_ExternalAHRS_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_Blimp mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_Blimp dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_TemperatureSensor_Blimp ${objs_AP_TemperatureSensor_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_TemperatureSensor_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_TemperatureSensor_Blimp PRIVATE ${objs_AP_TemperatureSensor_Blimp_INCLUDES})

set_target_properties(objs_AP_TemperatureSensor_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_Blimp mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_Blimp dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_AIS_Blimp ${objs_AP_AIS_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_AIS_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_AIS_Blimp PRIVATE ${objs_AP_AIS_Blimp_INCLUDES})

set_target_properties(objs_AP_AIS_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_Blimp mavlink) #clean
target_link_libraries(objs_AP_AIS_Blimp dronecan) #clean
target_link_libraries(objs_AP_AIS_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Arming_Blimp ${objs_AP_Arming_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Arming_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Arming_Blimp PRIVATE ${objs_AP_Arming_Blimp_INCLUDES})

set_target_properties(objs_AP_Arming_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_Blimp mavlink) #clean
target_link_libraries(objs_AP_Arming_Blimp dronecan) #clean
target_link_libraries(objs_AP_Arming_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Follow_Blimp ${objs_AP_Follow_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Follow_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Follow_Blimp PRIVATE ${objs_AP_Follow_Blimp_INCLUDES})

set_target_properties(objs_AP_Follow_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_Blimp mavlink) #clean
target_link_libraries(objs_AP_Follow_Blimp dronecan) #clean
target_link_libraries(objs_AP_Follow_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Avoidance_Blimp ${objs_AP_Avoidance_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Avoidance_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Avoidance_Blimp PRIVATE ${objs_AP_Avoidance_Blimp_INCLUDES})

set_target_properties(objs_AP_Avoidance_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Avoidance_Blimp mavlink) #clean
target_link_libraries(objs_AP_Avoidance_Blimp dronecan) #clean
target_link_libraries(objs_AP_Avoidance_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AC_AttitudeControl_Blimp ${objs_AC_AttitudeControl_Blimp_SOURCES}) #2
target_compile_definitions(objs_AC_AttitudeControl_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AC_AttitudeControl_Blimp PRIVATE ${objs_AC_AttitudeControl_Blimp_INCLUDES})

set_target_properties(objs_AC_AttitudeControl_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_Blimp mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_Blimp dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_HAL_SITL_Blimp ${objs_AP_HAL_SITL_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_SITL_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_HAL_SITL_Blimp PRIVATE ${objs_AP_HAL_SITL_Blimp_INCLUDES})

set_target_properties(objs_AP_HAL_SITL_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_Blimp mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_Blimp dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_SITL_Blimp ${objs_SITL_Blimp_SOURCES}) #2
target_compile_definitions(objs_SITL_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_SITL_Blimp PRIVATE ${objs_SITL_Blimp_INCLUDES})

set_target_properties(objs_SITL_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_Blimp mavlink) #clean
target_link_libraries(objs_SITL_Blimp dronecan) #clean
target_link_libraries(objs_SITL_Blimp littlefs) #clean

#------------------------------------------------

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

add_library(objs_AP_Scripting_Blimp ${objs_AP_Scripting_Blimp_SOURCES}) #2
target_compile_definitions(objs_AP_Scripting_Blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(objs_AP_Scripting_Blimp PRIVATE ${objs_AP_Scripting_Blimp_INCLUDES})

set_target_properties(objs_AP_Scripting_Blimp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_Blimp mavlink) #clean
target_link_libraries(objs_AP_Scripting_Blimp dronecan) #clean
target_link_libraries(objs_AP_Scripting_Blimp littlefs) #clean

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


target_link_libraries(Blimp_libs objs_AP_AccelCal_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_ADC_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_AHRS_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Airspeed_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Baro_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_BattMonitor_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_BoardConfig_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Camera_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Common_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Compass_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Declination_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_GPS_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_GSOF_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_HAL_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_HAL_Empty_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_DDS_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_InertialSensor_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Math_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Mission_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_DAL_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_NavEKF_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_NavEKF2_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_NavEKF3_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Notify_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_OpticalFlow_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Param_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Rally_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_LightWareSerial_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RangeFinder_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Scheduler_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_SerialManager_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Terrain_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Vehicle_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_InternalError_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Logger_Blimp) #stripped
target_link_libraries(Blimp_libs objs_Filter_Blimp) #stripped
target_link_libraries(Blimp_libs objs_GCS_MAVLink_Blimp) #stripped
target_link_libraries(Blimp_libs objs_RC_Channel_Blimp) #stripped
target_link_libraries(Blimp_libs objs_SRV_Channel_Blimp) #stripped
target_link_libraries(Blimp_libs objs_StorageManager_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Tuning_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RPM_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RSSI_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Mount_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Module_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Button_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_ICEngine_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Networking_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Frsky_Telem_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_IBus_Telem_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_FlashStorage_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Relay_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_ServoRelayEvents_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Volz_Protocol_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_SBusOut_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_IOMCU_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Parachute_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RAMTRON_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RCProtocol_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Radio_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_TempCalibration_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_VisualOdom_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_BLHeli_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_ROMFS_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Proximity_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Gripper_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RTC_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AC_Sprayer_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AC_Fence_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AC_Avoidance_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_LandingGear_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RobotisServo_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_NMEA_Output_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_OSD_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Filesystem_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_ADSB_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AC_PID_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_SerialLED_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_EFI_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Hott_Telem_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_ESC_Telem_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Servo_Telem_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Stats_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_GyroFFT_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RCTelemetry_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Generator_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_MSP_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_OLC_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_WheelEncoder_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_ExternalAHRS_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_VideoTX_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_FETtecOneWire_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_TemperatureSensor_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Torqeedo_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_CustomRotations_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_AIS_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_OpenDroneID_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_CheckFirmware_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_ExternalControl_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_JSON_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Beacon_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Arming_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_RCMapper_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_MultiHeap_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Follow_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_CANManager_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_KDECAN_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_PiccoloCAN_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AC_InputManager_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_InertialNav_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Avoidance_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_LTM_Telem_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Devo_Telem_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_AdvancedFailsafe_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AC_AttitudeControl_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_DroneCAN_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_HAL_SITL_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_CSVReader_Blimp) #stripped
target_link_libraries(Blimp_libs objs_SITL_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AP_Scripting_Blimp) #stripped
target_link_libraries(Blimp_libs objs_AC_CustomControl_Blimp) #stripped

#------------------------------------------------

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

add_executable(bin_blimp ${bin_blimp_SOURCES})
target_compile_definitions(bin_blimp PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_include_directories(bin_blimp PRIVATE ${bin_blimp_INCLUDES})


target_link_libraries(bin_blimp Blimp_libs) #clean

#------------------------------------------------

set(objs_AP_AHRS_Rover_SOURCES
    ../libraries/AP_AHRS/AP_AHRS.cpp
    ../libraries/AP_AHRS/AP_AHRS_Backend.cpp
    ../libraries/AP_AHRS/AP_AHRS_DCM.cpp)

set(objs_AP_AHRS_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_AHRS_Rover ${objs_AP_AHRS_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_AHRS_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_AHRS_Rover PRIVATE ${objs_AP_AHRS_Rover_INCLUDES})

set_target_properties(objs_AP_AHRS_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AHRS_Rover mavlink) #clean
target_link_libraries(objs_AP_AHRS_Rover dronecan) #clean
target_link_libraries(objs_AP_AHRS_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Airspeed_Rover_SOURCES
    ../libraries/AP_Airspeed/AP_Airspeed.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
    ../libraries/AP_Airspeed/AP_Airspeed_Params.cpp)

set(objs_AP_Airspeed_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Airspeed_Rover ${objs_AP_Airspeed_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Airspeed_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Airspeed_Rover PRIVATE ${objs_AP_Airspeed_Rover_INCLUDES})

set_target_properties(objs_AP_Airspeed_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Airspeed_Rover mavlink) #clean
target_link_libraries(objs_AP_Airspeed_Rover dronecan) #clean
target_link_libraries(objs_AP_Airspeed_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Baro_Rover_SOURCES
    ../libraries/AP_Baro/AP_Baro.cpp
    ../libraries/AP_Baro/AP_Baro_Backend.cpp
    ../libraries/AP_Baro/AP_Baro_SITL.cpp)

set(objs_AP_Baro_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Baro_Rover ${objs_AP_Baro_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Baro_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Baro_Rover PRIVATE ${objs_AP_Baro_Rover_INCLUDES})

set_target_properties(objs_AP_Baro_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Baro_Rover mavlink) #clean
target_link_libraries(objs_AP_Baro_Rover dronecan) #clean
target_link_libraries(objs_AP_Baro_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_BattMonitor_Rover_SOURCES
    ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    ../libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp)

set(objs_AP_BattMonitor_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_BattMonitor_Rover ${objs_AP_BattMonitor_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_BattMonitor_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_BattMonitor_Rover PRIVATE ${objs_AP_BattMonitor_Rover_INCLUDES})

set_target_properties(objs_AP_BattMonitor_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BattMonitor_Rover mavlink) #clean
target_link_libraries(objs_AP_BattMonitor_Rover dronecan) #clean
target_link_libraries(objs_AP_BattMonitor_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_BoardConfig_Rover_SOURCES
    ../libraries/AP_BoardConfig/AP_BoardConfig.cpp)

set(objs_AP_BoardConfig_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_BoardConfig_Rover ${objs_AP_BoardConfig_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_BoardConfig_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_BoardConfig_Rover PRIVATE ${objs_AP_BoardConfig_Rover_INCLUDES})

set_target_properties(objs_AP_BoardConfig_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BoardConfig_Rover mavlink) #clean
target_link_libraries(objs_AP_BoardConfig_Rover dronecan) #clean
target_link_libraries(objs_AP_BoardConfig_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_HAL_Rover_SOURCES
    ../libraries/AP_HAL/SIMState.cpp)

set(objs_AP_HAL_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_HAL_Rover ${objs_AP_HAL_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_HAL_Rover PRIVATE ${objs_AP_HAL_Rover_INCLUDES})

set_target_properties(objs_AP_HAL_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_Rover mavlink) #clean
target_link_libraries(objs_AP_HAL_Rover dronecan) #clean
target_link_libraries(objs_AP_HAL_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_InertialSensor_Rover_SOURCES
    ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
    ../libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
    ../libraries/AP_InertialSensor/FastRateBuffer.cpp)

set(objs_AP_InertialSensor_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_InertialSensor_Rover ${objs_AP_InertialSensor_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_InertialSensor_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_InertialSensor_Rover PRIVATE ${objs_AP_InertialSensor_Rover_INCLUDES})

set_target_properties(objs_AP_InertialSensor_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_InertialSensor_Rover mavlink) #clean
target_link_libraries(objs_AP_InertialSensor_Rover dronecan) #clean
target_link_libraries(objs_AP_InertialSensor_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Math_Rover_SOURCES
    ../libraries/AP_Math/SCurve.cpp)

set(objs_AP_Math_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Math_Rover ${objs_AP_Math_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Math_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Math_Rover PRIVATE ${objs_AP_Math_Rover_INCLUDES})

set_target_properties(objs_AP_Math_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Math_Rover mavlink) #clean
target_link_libraries(objs_AP_Math_Rover dronecan) #clean
target_link_libraries(objs_AP_Math_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Mission_Rover_SOURCES
    ../libraries/AP_Mission/AP_Mission.cpp)

set(objs_AP_Mission_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Mission_Rover ${objs_AP_Mission_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Mission_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Mission_Rover PRIVATE ${objs_AP_Mission_Rover_INCLUDES})

set_target_properties(objs_AP_Mission_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Mission_Rover mavlink) #clean
target_link_libraries(objs_AP_Mission_Rover dronecan) #clean
target_link_libraries(objs_AP_Mission_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_DAL_Rover_SOURCES
    ../libraries/AP_DAL/AP_DAL.cpp
    ../libraries/AP_DAL/AP_DAL_Beacon.cpp
    ../libraries/AP_DAL/AP_DAL_RangeFinder.cpp
    ../libraries/AP_DAL/AP_DAL_VisualOdom.cpp)

set(objs_AP_DAL_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_DAL_Rover ${objs_AP_DAL_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_DAL_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_DAL_Rover PRIVATE ${objs_AP_DAL_Rover_INCLUDES})

set_target_properties(objs_AP_DAL_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_DAL_Rover mavlink) #clean
target_link_libraries(objs_AP_DAL_Rover dronecan) #clean
target_link_libraries(objs_AP_DAL_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_NavEKF2_Rover_SOURCES
    ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    ../libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp)

set(objs_AP_NavEKF2_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_NavEKF2_Rover ${objs_AP_NavEKF2_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF2_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_NavEKF2_Rover PRIVATE ${objs_AP_NavEKF2_Rover_INCLUDES})

set_target_properties(objs_AP_NavEKF2_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF2_Rover mavlink) #clean
target_link_libraries(objs_AP_NavEKF2_Rover dronecan) #clean
target_link_libraries(objs_AP_NavEKF2_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_NavEKF3_Rover_SOURCES
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

set(objs_AP_NavEKF3_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_NavEKF3_Rover ${objs_AP_NavEKF3_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF3_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_NavEKF3_Rover PRIVATE ${objs_AP_NavEKF3_Rover_INCLUDES})

set_target_properties(objs_AP_NavEKF3_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3_Rover mavlink) #clean
target_link_libraries(objs_AP_NavEKF3_Rover dronecan) #clean
target_link_libraries(objs_AP_NavEKF3_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Rally_Rover_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs_AP_Rally_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Rally_Rover ${objs_AP_Rally_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Rally_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Rally_Rover PRIVATE ${objs_AP_Rally_Rover_INCLUDES})

set_target_properties(objs_AP_Rally_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally_Rover mavlink) #clean
target_link_libraries(objs_AP_Rally_Rover dronecan) #clean
target_link_libraries(objs_AP_Rally_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Scheduler_Rover_SOURCES
    ../libraries/AP_Scheduler/AP_Scheduler.cpp)

set(objs_AP_Scheduler_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Scheduler_Rover ${objs_AP_Scheduler_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Scheduler_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Scheduler_Rover PRIVATE ${objs_AP_Scheduler_Rover_INCLUDES})

set_target_properties(objs_AP_Scheduler_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scheduler_Rover mavlink) #clean
target_link_libraries(objs_AP_Scheduler_Rover dronecan) #clean
target_link_libraries(objs_AP_Scheduler_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Terrain_Rover_SOURCES
    ../libraries/AP_Terrain/AP_Terrain.cpp)

set(objs_AP_Terrain_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Terrain_Rover ${objs_AP_Terrain_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Terrain_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Terrain_Rover PRIVATE ${objs_AP_Terrain_Rover_INCLUDES})

set_target_properties(objs_AP_Terrain_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Terrain_Rover mavlink) #clean
target_link_libraries(objs_AP_Terrain_Rover dronecan) #clean
target_link_libraries(objs_AP_Terrain_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Vehicle_Rover_SOURCES
    ../libraries/AP_Vehicle/AP_Vehicle.cpp)

set(objs_AP_Vehicle_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Vehicle_Rover ${objs_AP_Vehicle_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Vehicle_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Vehicle_Rover PRIVATE ${objs_AP_Vehicle_Rover_INCLUDES})

set_target_properties(objs_AP_Vehicle_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Vehicle_Rover mavlink) #clean
target_link_libraries(objs_AP_Vehicle_Rover dronecan) #clean
target_link_libraries(objs_AP_Vehicle_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Logger_Rover_SOURCES
    ../libraries/AP_Logger/AP_Logger.cpp
    ../libraries/AP_Logger/AP_Logger_Backend.cpp
    ../libraries/AP_Logger/AP_Logger_File.cpp
    ../libraries/AP_Logger/LoggerMessageWriter.cpp)

set(objs_AP_Logger_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Logger_Rover ${objs_AP_Logger_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Logger_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Logger_Rover PRIVATE ${objs_AP_Logger_Rover_INCLUDES})

set_target_properties(objs_AP_Logger_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Logger_Rover mavlink) #clean
target_link_libraries(objs_AP_Logger_Rover dronecan) #clean
target_link_libraries(objs_AP_Logger_Rover littlefs) #clean

#------------------------------------------------

set(objs_Filter_Rover_SOURCES
    ../libraries/Filter/HarmonicNotchFilter.cpp)

set(objs_Filter_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_Filter_Rover ${objs_Filter_Rover_SOURCES}) #2
target_compile_definitions(objs_Filter_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_Filter_Rover PRIVATE ${objs_Filter_Rover_INCLUDES})

set_target_properties(objs_Filter_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_Filter_Rover mavlink) #clean
target_link_libraries(objs_Filter_Rover dronecan) #clean
target_link_libraries(objs_Filter_Rover littlefs) #clean

#------------------------------------------------

set(objs_GCS_MAVLink_Rover_SOURCES
    ../libraries/GCS_MAVLink/GCS.cpp
    ../libraries/GCS_MAVLink/GCS_Common.cpp
    ../libraries/GCS_MAVLink/GCS_Dummy.cpp
    ../libraries/GCS_MAVLink/GCS_Fence.cpp
    ../libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp)

set(objs_GCS_MAVLink_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_GCS_MAVLink_Rover ${objs_GCS_MAVLink_Rover_SOURCES}) #2
target_compile_definitions(objs_GCS_MAVLink_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_GCS_MAVLink_Rover PRIVATE ${objs_GCS_MAVLink_Rover_INCLUDES})

set_target_properties(objs_GCS_MAVLink_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_GCS_MAVLink_Rover mavlink) #clean
target_link_libraries(objs_GCS_MAVLink_Rover dronecan) #clean
target_link_libraries(objs_GCS_MAVLink_Rover littlefs) #clean

#------------------------------------------------

set(objs_RC_Channel_Rover_SOURCES
    ../libraries/RC_Channel/RC_Channel.cpp)

set(objs_RC_Channel_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_RC_Channel_Rover ${objs_RC_Channel_Rover_SOURCES}) #2
target_compile_definitions(objs_RC_Channel_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_RC_Channel_Rover PRIVATE ${objs_RC_Channel_Rover_INCLUDES})

set_target_properties(objs_RC_Channel_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_RC_Channel_Rover mavlink) #clean
target_link_libraries(objs_RC_Channel_Rover dronecan) #clean
target_link_libraries(objs_RC_Channel_Rover littlefs) #clean

#------------------------------------------------

set(objs_StorageManager_Rover_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs_StorageManager_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_StorageManager_Rover ${objs_StorageManager_Rover_SOURCES}) #2
target_compile_definitions(objs_StorageManager_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_StorageManager_Rover PRIVATE ${objs_StorageManager_Rover_INCLUDES})

set_target_properties(objs_StorageManager_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager_Rover mavlink) #clean
target_link_libraries(objs_StorageManager_Rover dronecan) #clean
target_link_libraries(objs_StorageManager_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Frsky_Telem_Rover_SOURCES
    ../libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp)

set(objs_AP_Frsky_Telem_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Frsky_Telem_Rover ${objs_AP_Frsky_Telem_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Frsky_Telem_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Frsky_Telem_Rover PRIVATE ${objs_AP_Frsky_Telem_Rover_INCLUDES})

set_target_properties(objs_AP_Frsky_Telem_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Frsky_Telem_Rover mavlink) #clean
target_link_libraries(objs_AP_Frsky_Telem_Rover dronecan) #clean
target_link_libraries(objs_AP_Frsky_Telem_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_IBus_Telem_Rover_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs_AP_IBus_Telem_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_IBus_Telem_Rover ${objs_AP_IBus_Telem_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_IBus_Telem_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_IBus_Telem_Rover PRIVATE ${objs_AP_IBus_Telem_Rover_INCLUDES})

set_target_properties(objs_AP_IBus_Telem_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem_Rover mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem_Rover dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Relay_Rover_SOURCES
    ../libraries/AP_Relay/AP_Relay.cpp)

set(objs_AP_Relay_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Relay_Rover ${objs_AP_Relay_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Relay_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Relay_Rover PRIVATE ${objs_AP_Relay_Rover_INCLUDES})

set_target_properties(objs_AP_Relay_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Relay_Rover mavlink) #clean
target_link_libraries(objs_AP_Relay_Rover dronecan) #clean
target_link_libraries(objs_AP_Relay_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_RCProtocol_Rover_SOURCES
    ../libraries/AP_RCProtocol/AP_RCProtocol.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
    ../libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp)

set(objs_AP_RCProtocol_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_RCProtocol_Rover ${objs_AP_RCProtocol_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_RCProtocol_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_RCProtocol_Rover PRIVATE ${objs_AP_RCProtocol_Rover_INCLUDES})

set_target_properties(objs_AP_RCProtocol_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCProtocol_Rover mavlink) #clean
target_link_libraries(objs_AP_RCProtocol_Rover dronecan) #clean
target_link_libraries(objs_AP_RCProtocol_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_BLHeli_Rover_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs_AP_BLHeli_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_BLHeli_Rover ${objs_AP_BLHeli_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_BLHeli_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_BLHeli_Rover PRIVATE ${objs_AP_BLHeli_Rover_INCLUDES})

set_target_properties(objs_AP_BLHeli_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli_Rover mavlink) #clean
target_link_libraries(objs_AP_BLHeli_Rover dronecan) #clean
target_link_libraries(objs_AP_BLHeli_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Proximity_Rover_SOURCES
    ../libraries/AP_Proximity/AP_Proximity_Utils.cpp)

set(objs_AP_Proximity_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Proximity_Rover ${objs_AP_Proximity_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Proximity_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Proximity_Rover PRIVATE ${objs_AP_Proximity_Rover_INCLUDES})

set_target_properties(objs_AP_Proximity_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Proximity_Rover mavlink) #clean
target_link_libraries(objs_AP_Proximity_Rover dronecan) #clean
target_link_libraries(objs_AP_Proximity_Rover littlefs) #clean

#------------------------------------------------

set(objs_AC_Fence_Rover_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs_AC_Fence_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AC_Fence_Rover ${objs_AC_Fence_Rover_SOURCES}) #2
target_compile_definitions(objs_AC_Fence_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AC_Fence_Rover PRIVATE ${objs_AC_Fence_Rover_INCLUDES})

set_target_properties(objs_AC_Fence_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence_Rover mavlink) #clean
target_link_libraries(objs_AC_Fence_Rover dronecan) #clean
target_link_libraries(objs_AC_Fence_Rover littlefs) #clean

#------------------------------------------------

set(objs_AC_Avoidance_Rover_SOURCES
    ../libraries/AC_Avoidance/AC_Avoid.cpp
    ../libraries/AC_Avoidance/AP_OABendyRuler.cpp
    ../libraries/AC_Avoidance/AP_OADatabase.cpp)

set(objs_AC_Avoidance_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AC_Avoidance_Rover ${objs_AC_Avoidance_Rover_SOURCES}) #2
target_compile_definitions(objs_AC_Avoidance_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AC_Avoidance_Rover PRIVATE ${objs_AC_Avoidance_Rover_INCLUDES})

set_target_properties(objs_AC_Avoidance_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Avoidance_Rover mavlink) #clean
target_link_libraries(objs_AC_Avoidance_Rover dronecan) #clean
target_link_libraries(objs_AC_Avoidance_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_LandingGear_Rover_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs_AP_LandingGear_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_LandingGear_Rover ${objs_AP_LandingGear_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_LandingGear_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_LandingGear_Rover PRIVATE ${objs_AP_LandingGear_Rover_INCLUDES})

set_target_properties(objs_AP_LandingGear_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear_Rover mavlink) #clean
target_link_libraries(objs_AP_LandingGear_Rover dronecan) #clean
target_link_libraries(objs_AP_LandingGear_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_OSD_Rover_SOURCES
    ../libraries/AP_OSD/AP_OSD_ParamScreen.cpp
    ../libraries/AP_OSD/AP_OSD_ParamSetting.cpp
    ../libraries/AP_OSD/AP_OSD_Screen.cpp)

set(objs_AP_OSD_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_OSD_Rover ${objs_AP_OSD_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_OSD_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_OSD_Rover PRIVATE ${objs_AP_OSD_Rover_INCLUDES})

set_target_properties(objs_AP_OSD_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_OSD_Rover mavlink) #clean
target_link_libraries(objs_AP_OSD_Rover dronecan) #clean
target_link_libraries(objs_AP_OSD_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Filesystem_Rover_SOURCES
    ../libraries/AP_Filesystem/AP_Filesystem_posix.cpp)

set(objs_AP_Filesystem_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Filesystem_Rover ${objs_AP_Filesystem_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Filesystem_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Filesystem_Rover PRIVATE ${objs_AP_Filesystem_Rover_INCLUDES})

set_target_properties(objs_AP_Filesystem_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Filesystem_Rover mavlink) #clean
target_link_libraries(objs_AP_Filesystem_Rover dronecan) #clean
target_link_libraries(objs_AP_Filesystem_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_ADSB_Rover_SOURCES
    ../libraries/AP_ADSB/AP_ADSB.cpp
    ../libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp)

set(objs_AP_ADSB_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_ADSB_Rover ${objs_AP_ADSB_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_ADSB_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_ADSB_Rover PRIVATE ${objs_AP_ADSB_Rover_INCLUDES})

set_target_properties(objs_AP_ADSB_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ADSB_Rover mavlink) #clean
target_link_libraries(objs_AP_ADSB_Rover dronecan) #clean
target_link_libraries(objs_AP_ADSB_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_ESC_Telem_Rover_SOURCES
    ../libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp)

set(objs_AP_ESC_Telem_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_ESC_Telem_Rover ${objs_AP_ESC_Telem_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_ESC_Telem_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_ESC_Telem_Rover PRIVATE ${objs_AP_ESC_Telem_Rover_INCLUDES})

set_target_properties(objs_AP_ESC_Telem_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ESC_Telem_Rover mavlink) #clean
target_link_libraries(objs_AP_ESC_Telem_Rover dronecan) #clean
target_link_libraries(objs_AP_ESC_Telem_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_GyroFFT_Rover_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs_AP_GyroFFT_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_GyroFFT_Rover ${objs_AP_GyroFFT_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_GyroFFT_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_GyroFFT_Rover PRIVATE ${objs_AP_GyroFFT_Rover_INCLUDES})

set_target_properties(objs_AP_GyroFFT_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT_Rover mavlink) #clean
target_link_libraries(objs_AP_GyroFFT_Rover dronecan) #clean
target_link_libraries(objs_AP_GyroFFT_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_RCTelemetry_Rover_SOURCES
    ../libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
    ../libraries/AP_RCTelemetry/AP_RCTelemetry.cpp)

set(objs_AP_RCTelemetry_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_RCTelemetry_Rover ${objs_AP_RCTelemetry_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_RCTelemetry_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_RCTelemetry_Rover PRIVATE ${objs_AP_RCTelemetry_Rover_INCLUDES})

set_target_properties(objs_AP_RCTelemetry_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_RCTelemetry_Rover mavlink) #clean
target_link_libraries(objs_AP_RCTelemetry_Rover dronecan) #clean
target_link_libraries(objs_AP_RCTelemetry_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_ExternalAHRS_Rover_SOURCES
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
    ../libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp)

set(objs_AP_ExternalAHRS_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_ExternalAHRS_Rover ${objs_AP_ExternalAHRS_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_ExternalAHRS_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_ExternalAHRS_Rover PRIVATE ${objs_AP_ExternalAHRS_Rover_INCLUDES})

set_target_properties(objs_AP_ExternalAHRS_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_ExternalAHRS_Rover mavlink) #clean
target_link_libraries(objs_AP_ExternalAHRS_Rover dronecan) #clean
target_link_libraries(objs_AP_ExternalAHRS_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_TemperatureSensor_Rover_SOURCES
    ../libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp)

set(objs_AP_TemperatureSensor_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_TemperatureSensor_Rover ${objs_AP_TemperatureSensor_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_TemperatureSensor_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_TemperatureSensor_Rover PRIVATE ${objs_AP_TemperatureSensor_Rover_INCLUDES})

set_target_properties(objs_AP_TemperatureSensor_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_TemperatureSensor_Rover mavlink) #clean
target_link_libraries(objs_AP_TemperatureSensor_Rover dronecan) #clean
target_link_libraries(objs_AP_TemperatureSensor_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_AIS_Rover_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs_AP_AIS_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_AIS_Rover ${objs_AP_AIS_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_AIS_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_AIS_Rover PRIVATE ${objs_AP_AIS_Rover_INCLUDES})

set_target_properties(objs_AP_AIS_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS_Rover mavlink) #clean
target_link_libraries(objs_AP_AIS_Rover dronecan) #clean
target_link_libraries(objs_AP_AIS_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Arming_Rover_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs_AP_Arming_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Arming_Rover ${objs_AP_Arming_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Arming_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Arming_Rover PRIVATE ${objs_AP_Arming_Rover_INCLUDES})

set_target_properties(objs_AP_Arming_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming_Rover mavlink) #clean
target_link_libraries(objs_AP_Arming_Rover dronecan) #clean
target_link_libraries(objs_AP_Arming_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Follow_Rover_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs_AP_Follow_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Follow_Rover ${objs_AP_Follow_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Follow_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Follow_Rover PRIVATE ${objs_AP_Follow_Rover_INCLUDES})

set_target_properties(objs_AP_Follow_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow_Rover mavlink) #clean
target_link_libraries(objs_AP_Follow_Rover dronecan) #clean
target_link_libraries(objs_AP_Follow_Rover littlefs) #clean

#------------------------------------------------

set(objs_AC_AttitudeControl_Rover_SOURCES
    ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp)

set(objs_AC_AttitudeControl_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AC_AttitudeControl_Rover ${objs_AC_AttitudeControl_Rover_SOURCES}) #2
target_compile_definitions(objs_AC_AttitudeControl_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AC_AttitudeControl_Rover PRIVATE ${objs_AC_AttitudeControl_Rover_INCLUDES})

set_target_properties(objs_AC_AttitudeControl_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_AttitudeControl_Rover mavlink) #clean
target_link_libraries(objs_AC_AttitudeControl_Rover dronecan) #clean
target_link_libraries(objs_AC_AttitudeControl_Rover littlefs) #clean

#------------------------------------------------

set(objs_AC_PrecLand_Rover_SOURCES
    ../libraries/AC_PrecLand/AC_PrecLand.cpp)

set(objs_AC_PrecLand_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AC_PrecLand_Rover ${objs_AC_PrecLand_Rover_SOURCES}) #2
target_compile_definitions(objs_AC_PrecLand_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AC_PrecLand_Rover PRIVATE ${objs_AC_PrecLand_Rover_INCLUDES})

set_target_properties(objs_AC_PrecLand_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_PrecLand_Rover mavlink) #clean
target_link_libraries(objs_AC_PrecLand_Rover dronecan) #clean
target_link_libraries(objs_AC_PrecLand_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_HAL_SITL_Rover_SOURCES
    ../libraries/AP_HAL_SITL/SITL_cmdline.cpp
    ../libraries/AP_HAL_SITL/Storage.cpp
    ../libraries/AP_HAL_SITL/UARTDriver.cpp)

set(objs_AP_HAL_SITL_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_HAL_SITL_Rover ${objs_AP_HAL_SITL_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_HAL_SITL_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_HAL_SITL_Rover PRIVATE ${objs_AP_HAL_SITL_Rover_INCLUDES})

set_target_properties(objs_AP_HAL_SITL_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_HAL_SITL_Rover mavlink) #clean
target_link_libraries(objs_AP_HAL_SITL_Rover dronecan) #clean
target_link_libraries(objs_AP_HAL_SITL_Rover littlefs) #clean

#------------------------------------------------

set(objs_SITL_Rover_SOURCES
    ../libraries/SITL/SIM_Aircraft.cpp
    ../libraries/SITL/SIM_DroneCANDevice.cpp
    ../libraries/SITL/SIM_XPlane.cpp
    ../libraries/SITL/SITL.cpp)

set(objs_SITL_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_SITL_Rover ${objs_SITL_Rover_SOURCES}) #2
target_compile_definitions(objs_SITL_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_SITL_Rover PRIVATE ${objs_SITL_Rover_INCLUDES})

set_target_properties(objs_SITL_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_SITL_Rover mavlink) #clean
target_link_libraries(objs_SITL_Rover dronecan) #clean
target_link_libraries(objs_SITL_Rover littlefs) #clean

#------------------------------------------------

set(objs_AP_Scripting_Rover_SOURCES
    ../libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
    ../libraries/AP_Scripting/AP_Scripting_helpers.cpp
    ../libraries/AP_Scripting/lua_bindings.cpp
    ../libraries/AP_Scripting/lua_boxed_numerics.cpp
    ../libraries/AP_Scripting/lua_scripts.cpp
    ../build/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp)

set(objs_AP_Scripting_Rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Scripting_Rover ${objs_AP_Scripting_Rover_SOURCES}) #2
target_compile_definitions(objs_AP_Scripting_Rover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(objs_AP_Scripting_Rover PRIVATE ${objs_AP_Scripting_Rover_INCLUDES})

set_target_properties(objs_AP_Scripting_Rover PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Scripting_Rover mavlink) #clean
target_link_libraries(objs_AP_Scripting_Rover dronecan) #clean
target_link_libraries(objs_AP_Scripting_Rover littlefs) #clean

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


target_link_libraries(Rover_libs objs_AP_AccelCal_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_ADC_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_AHRS_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Airspeed_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Baro_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_BattMonitor_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_BoardConfig_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Camera_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Common_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Compass_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Declination_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_GPS_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_GSOF_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_HAL_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_HAL_Empty_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_DDS_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_InertialSensor_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Math_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Mission_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_DAL_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_NavEKF_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_NavEKF2_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_NavEKF3_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Notify_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_OpticalFlow_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Param_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Rally_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_LightWareSerial_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RangeFinder_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Scheduler_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_SerialManager_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Terrain_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Vehicle_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_InternalError_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Logger_Rover) #stripped
target_link_libraries(Rover_libs objs_Filter_Rover) #stripped
target_link_libraries(Rover_libs objs_GCS_MAVLink_Rover) #stripped
target_link_libraries(Rover_libs objs_RC_Channel_Rover) #stripped
target_link_libraries(Rover_libs objs_SRV_Channel_Rover) #stripped
target_link_libraries(Rover_libs objs_StorageManager_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Tuning_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RPM_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RSSI_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Mount_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Module_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Button_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_ICEngine_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Networking_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Frsky_Telem_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_IBus_Telem_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_FlashStorage_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Relay_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_ServoRelayEvents_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Volz_Protocol_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_SBusOut_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_IOMCU_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Parachute_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RAMTRON_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RCProtocol_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Radio_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_TempCalibration_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_VisualOdom_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_BLHeli_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_ROMFS_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Proximity_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Gripper_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RTC_Rover) #stripped
target_link_libraries(Rover_libs objs_AC_Sprayer_Rover) #stripped
target_link_libraries(Rover_libs objs_AC_Fence_Rover) #stripped
target_link_libraries(Rover_libs objs_AC_Avoidance_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_LandingGear_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RobotisServo_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_NMEA_Output_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_OSD_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Filesystem_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_ADSB_Rover) #stripped
target_link_libraries(Rover_libs objs_AC_PID_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_SerialLED_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_EFI_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Hott_Telem_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_ESC_Telem_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Servo_Telem_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Stats_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_GyroFFT_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RCTelemetry_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Generator_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_MSP_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_OLC_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_WheelEncoder_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_ExternalAHRS_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_VideoTX_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_FETtecOneWire_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_TemperatureSensor_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Torqeedo_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_CustomRotations_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_AIS_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_OpenDroneID_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_CheckFirmware_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_ExternalControl_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_JSON_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Beacon_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Arming_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_RCMapper_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_MultiHeap_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Follow_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_CANManager_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_KDECAN_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_PiccoloCAN_Rover) #stripped
target_link_libraries(Rover_libs objs_APM_Control_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Navigation_Rover) #stripped
target_link_libraries(Rover_libs objs_AR_WPNav_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_AdvancedFailsafe_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_SmartRTL_Rover) #stripped
target_link_libraries(Rover_libs objs_AC_AttitudeControl_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_LTM_Telem_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Devo_Telem_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_WindVane_Rover) #stripped
target_link_libraries(Rover_libs objs_AR_Motors_Rover) #stripped
target_link_libraries(Rover_libs objs_AC_PrecLand_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_IRLock_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_DroneCAN_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_HAL_SITL_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_CSVReader_Rover) #stripped
target_link_libraries(Rover_libs objs_SITL_Rover) #stripped
target_link_libraries(Rover_libs objs_AP_Scripting_Rover) #stripped
target_link_libraries(Rover_libs objs_AC_CustomControl_Rover) #stripped

#------------------------------------------------

set(bin_ardurover_SOURCES
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
target_compile_definitions(bin_ardurover PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_include_directories(bin_ardurover PRIVATE ${bin_ardurover_INCLUDES})


target_link_libraries(bin_ardurover Rover_libs) #clean

