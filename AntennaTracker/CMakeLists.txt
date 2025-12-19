
#------------------------------------------------

set(objs_AP_NavEKF3_SOURCES
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

set(objs_AP_NavEKF3_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_NavEKF3 ${objs_AP_NavEKF3_SOURCES}) #2
target_compile_definitions(objs_AP_NavEKF3 PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_NavEKF3 PRIVATE ${objs_AP_NavEKF3_INCLUDES})

set_target_properties(objs_AP_NavEKF3 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_NavEKF3 mavlink) #clean
target_link_libraries(objs_AP_NavEKF3 dronecan) #clean
target_link_libraries(objs_AP_NavEKF3 littlefs) #clean

#------------------------------------------------

set(objs_AP_Rally_SOURCES
    ../libraries/AP_Rally/AP_Rally.cpp)

set(objs_AP_Rally_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Rally ${objs_AP_Rally_SOURCES}) #2
target_compile_definitions(objs_AP_Rally PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Rally PRIVATE ${objs_AP_Rally_INCLUDES})

set_target_properties(objs_AP_Rally PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Rally mavlink) #clean
target_link_libraries(objs_AP_Rally dronecan) #clean
target_link_libraries(objs_AP_Rally littlefs) #clean

#------------------------------------------------

set(objs_StorageManager_SOURCES
    ../libraries/StorageManager/StorageManager.cpp)

set(objs_StorageManager_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_StorageManager ${objs_StorageManager_SOURCES}) #2
target_compile_definitions(objs_StorageManager PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_StorageManager PRIVATE ${objs_StorageManager_INCLUDES})

set_target_properties(objs_StorageManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_StorageManager mavlink) #clean
target_link_libraries(objs_StorageManager dronecan) #clean
target_link_libraries(objs_StorageManager littlefs) #clean

#------------------------------------------------

set(objs_AP_IBus_Telem_SOURCES
    ../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp)

set(objs_AP_IBus_Telem_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_IBus_Telem ${objs_AP_IBus_Telem_SOURCES}) #2
target_compile_definitions(objs_AP_IBus_Telem PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_IBus_Telem PRIVATE ${objs_AP_IBus_Telem_INCLUDES})

set_target_properties(objs_AP_IBus_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_IBus_Telem mavlink) #clean
target_link_libraries(objs_AP_IBus_Telem dronecan) #clean
target_link_libraries(objs_AP_IBus_Telem littlefs) #clean

#------------------------------------------------

set(objs_AP_BLHeli_SOURCES
    ../libraries/AP_BLHeli/AP_BLHeli.cpp)

set(objs_AP_BLHeli_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_BLHeli ${objs_AP_BLHeli_SOURCES}) #2
target_compile_definitions(objs_AP_BLHeli PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_BLHeli PRIVATE ${objs_AP_BLHeli_INCLUDES})

set_target_properties(objs_AP_BLHeli PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_BLHeli mavlink) #clean
target_link_libraries(objs_AP_BLHeli dronecan) #clean
target_link_libraries(objs_AP_BLHeli littlefs) #clean

#------------------------------------------------

set(objs_AC_Fence_SOURCES
    ../libraries/AC_Fence/AC_Fence.cpp
    ../libraries/AC_Fence/AC_PolyFence_loader.cpp)

set(objs_AC_Fence_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AC_Fence ${objs_AC_Fence_SOURCES}) #2
target_compile_definitions(objs_AC_Fence PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AC_Fence PRIVATE ${objs_AC_Fence_INCLUDES})

set_target_properties(objs_AC_Fence PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AC_Fence mavlink) #clean
target_link_libraries(objs_AC_Fence dronecan) #clean
target_link_libraries(objs_AC_Fence littlefs) #clean

#------------------------------------------------

set(objs_AP_LandingGear_SOURCES
    ../libraries/AP_LandingGear/AP_LandingGear.cpp)

set(objs_AP_LandingGear_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_LandingGear ${objs_AP_LandingGear_SOURCES}) #2
target_compile_definitions(objs_AP_LandingGear PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_LandingGear PRIVATE ${objs_AP_LandingGear_INCLUDES})

set_target_properties(objs_AP_LandingGear PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_LandingGear mavlink) #clean
target_link_libraries(objs_AP_LandingGear dronecan) #clean
target_link_libraries(objs_AP_LandingGear littlefs) #clean

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

set(objs_AP_GyroFFT_SOURCES
    ../libraries/AP_GyroFFT/AP_GyroFFT.cpp)

set(objs_AP_GyroFFT_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_GyroFFT ${objs_AP_GyroFFT_SOURCES}) #2
target_compile_definitions(objs_AP_GyroFFT PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_GyroFFT PRIVATE ${objs_AP_GyroFFT_INCLUDES})

set_target_properties(objs_AP_GyroFFT PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_GyroFFT mavlink) #clean
target_link_libraries(objs_AP_GyroFFT dronecan) #clean
target_link_libraries(objs_AP_GyroFFT littlefs) #clean

#------------------------------------------------

set(objs_AP_AIS_SOURCES
    ../libraries/AP_AIS/AP_AIS.cpp)

set(objs_AP_AIS_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_AIS ${objs_AP_AIS_SOURCES}) #2
target_compile_definitions(objs_AP_AIS PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_AIS PRIVATE ${objs_AP_AIS_INCLUDES})

set_target_properties(objs_AP_AIS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_AIS mavlink) #clean
target_link_libraries(objs_AP_AIS dronecan) #clean
target_link_libraries(objs_AP_AIS littlefs) #clean

#------------------------------------------------

set(objs_AP_Arming_SOURCES
    ../libraries/AP_Arming/AP_Arming.cpp)

set(objs_AP_Arming_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Arming ${objs_AP_Arming_SOURCES}) #2
target_compile_definitions(objs_AP_Arming PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Arming PRIVATE ${objs_AP_Arming_INCLUDES})

set_target_properties(objs_AP_Arming PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Arming mavlink) #clean
target_link_libraries(objs_AP_Arming dronecan) #clean
target_link_libraries(objs_AP_Arming littlefs) #clean

#------------------------------------------------

set(objs_AP_Follow_SOURCES
    ../libraries/AP_Follow/AP_Follow.cpp)

set(objs_AP_Follow_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Follow ${objs_AP_Follow_SOURCES}) #2
target_compile_definitions(objs_AP_Follow PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_include_directories(objs_AP_Follow PRIVATE ${objs_AP_Follow_INCLUDES})

set_target_properties(objs_AP_Follow PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Follow mavlink) #clean
target_link_libraries(objs_AP_Follow dronecan) #clean
target_link_libraries(objs_AP_Follow littlefs) #clean

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


target_link_libraries(AntennaTracker_libs ../libobjs_AP_AccelCal.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_ADC.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_AHRS.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Airspeed.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Baro.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_BattMonitor.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_BoardConfig.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Camera.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Common.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Compass.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Declination.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_GPS.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_GSOF.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_HAL.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_HAL_Empty.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_DDS.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_InertialSensor.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Math.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Mission.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_DAL.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_NavEKF.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_NavEKF2.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_NavEKF3.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Notify.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_OpticalFlow.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Param.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Rally.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_LightWareSerial.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RangeFinder.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Scheduler.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_SerialManager.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Terrain.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Vehicle.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_InternalError.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Logger.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_Filter.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_GCS_MAVLink.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_RC_Channel.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_SRV_Channel.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_StorageManager.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Tuning.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RPM.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RSSI.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Mount.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Module.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Button.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_ICEngine.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Networking.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Frsky_Telem.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_IBus_Telem.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_FlashStorage.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Relay.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_ServoRelayEvents.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Volz_Protocol.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_SBusOut.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_IOMCU.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Parachute.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RAMTRON.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RCProtocol.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Radio.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_TempCalibration.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_VisualOdom.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_BLHeli.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_ROMFS.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Proximity.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Gripper.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RTC.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AC_Sprayer.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AC_Fence.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AC_Avoidance.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_LandingGear.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RobotisServo.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_NMEA_Output.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_OSD.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Filesystem.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_ADSB.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AC_PID.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_SerialLED.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_EFI.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Hott_Telem.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_ESC_Telem.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Servo_Telem.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Stats.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_GyroFFT.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RCTelemetry.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Generator.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_MSP.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_OLC.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_WheelEncoder.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_ExternalAHRS.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_VideoTX.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_FETtecOneWire.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_TemperatureSensor.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Torqeedo.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_CustomRotations.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_AIS.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_OpenDroneID.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_CheckFirmware.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_ExternalControl.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_JSON.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Beacon.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Arming.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_RCMapper.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_MultiHeap.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Follow.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_CANManager.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_KDECAN.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_PiccoloCAN.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_DroneCAN.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_HAL_SITL.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_CSVReader.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_SITL.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AP_Scripting.a) #stripped2
target_link_libraries(AntennaTracker_libs ../libobjs_AC_CustomControl.a) #stripped2

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

