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
"${CMAKE_BINARY_DIR}/sitl"
${CMAKE_CURRENT_SOURCE_DIR}/modules/DroneCAN/libcanard
${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_DroneCAN/canard 
"${CMAKE_BINARY_DIR}/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include" 
)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_definitions(-DAP_BARO_PROBE_EXTERNAL_I2C_BUSES=1 -DAP_CUSTOMCONTROL_ENABLED=1 -DAP_DDS_ENABLED=0 -DAP_SCRIPTING_CHECKS=1 -DCANARD_ALLOCATE_SEM=1 -DCANARD_ENABLE_ASSERTS=1 -DCANARD_ENABLE_CANFD=1 -DCANARD_ENABLE_DEADLINE=1 -DCANARD_IFACE_ALL=3 -DCANARD_MULTI_IFACE=1 -DCONFIG_HAL_BOARD=HAL_BOARD_SITL -DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE -DDRONECAN_CXX_WRAPPERS=1 -DENABLE_ONVIF=0 -DLUA_32BITS=1 -DUSE_USER_HELPERS=1 -DAPM_BUILD_DIRECTORY=2) #1

set(CMAKE_C_FLAGS "-ffunction-sections -fdata-sections -fsigned-char -Wall -Wextra -Werror=format -Wpointer-arith -Wcast-align -Wno-missing-field-initializers -Wno-unused-parameter -Wno-redundant-decls -Wno-unknown-pragmas -Wno-trigraphs -Werror=return-type -Werror=unused-result -Werror=unused-variable -Werror=narrowing -Werror=attributes -Werror=overflow -Werror=parentheses -Werror=format-extra-args -Werror=ignored-qualifiers -Werror=undef -DARDUPILOT_BUILD -Wno-format-contains-nul -fsingle-precision-constant -Werror=use-after-free -DCANARD_64_BIT=1 -MMD -ffile-prefix-map=../..=../..")
set(CMAKE_CXX_FLAGS "-std=gnu++11 -fdata-sections -ffunction-sections -fno-exceptions -fsigned-char -Wall -Wextra -Wpointer-arith -Wno-unused-parameter -Wno-missing-field-initializers -Wno-redundant-decls -Wno-unknown-pragmas -Wno-expansion-to-defined -Werror=reorder -Werror=cast-align -Werror=attributes -Werror=format-security -Werror=format-extra-args -Werror=enum-compare -Werror=format -Werror=array-bounds -Werror=uninitialized -Werror=init-self -Werror=narrowing -Werror=return-type -Werror=switch -Werror=sign-compare -Werror=type-limits -Werror=undef -Werror=unused-result -Werror=unused-value -Werror=unused-variable -Werror=delete-non-virtual-dtor -Wfatal-errors -Wno-trigraphs -Werror=parentheses -DARDUPILOT_BUILD -Wuninitialized -Warray-bounds -Wno-format-contains-nul -Werror=unused-but-set-variable -fsingle-precision-constant -Wno-psabi -Werror=suggest-override -Werror=implicit-fallthrough -Werror=maybe-uninitialized -Werror=duplicated-cond -Werror=sizeof-pointer-div -Werror=use-after-free -D__AP_LINE__=__LINE__ -DAP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED=HAL_GCS_ENABLED -DAC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT=HAL_GCS_ENABLED -DHAL_NAVEKF2_AVAILABLE=1 -DCANARD_64_BIT=1 -Werror=float-equal -Werror=missing-declarations -DAP_NETWORKING_ENABLED=1 -O3 -MMD -ffile-prefix-map=../..=../.. -DHAL_HAVE_AP_ROMFS_EMBEDDED_H  -D_AP_CONFIG_H_=1 -DWAF_BUILD=1 -D__STDC_FORMAT_MACROS=1 -DAP_SIM_ENABLED=1 -DHAL_WITH_SPI=1 -DHAL_WITH_RAMTRON=1 -DAP_OPENDRONEID_ENABLED=1 -DAP_SIGNED_FIRMWARE=0 -DAP_NOTIFY_LP5562_BUS=2 -DAP_NOTIFY_LP5562_ADDR=48 -DHAL_NUM_CAN_IFACES=2 -DHAL_CAN_WITH_SOCKETCAN=1 -DHAVE_FEENABLEEXCEPT=1 -DHAVE_CMATH_ISFINITE=1 -DHAVE_CMATH_ISINF=1 -DHAVE_CMATH_ISNAN=1 -DNEED_CMATH_ISFINITE_STD_NAMESPACE=1 -DNEED_CMATH_ISINF_STD_NAMESPACE=1 -DNEED_CMATH_ISNAN_STD_NAMESPACE=1 -DHAVE_ENDIAN_H=1 -DHAVE_BYTESWAP_H=1 -DHAVE_MEMRCHR=1 -D_GNU_SOURCE=1 ")

# Build the gen-bindings tool
add_executable(gen-bindings libraries/AP_Scripting/generator/src/main.c)
target_compile_options(gen-bindings PRIVATE -std=c99 -Wno-error=missing-field-initializers -Wall -Wextra -Wno-error=maybe-uninitialized)
# Generate lua_generated_bindings files
add_custom_command(
	OUTPUT ${CMAKE_BINARY_DIR}/libraries/AP_Scripting/lua_generated_bindings.cpp ${CMAKE_BINARY_DIR}/libraries/AP_Scripting/lua_generated_bindings.h
	COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/libraries/AP_Scripting
	COMMAND gen-bindings -o ${CMAKE_BINARY_DIR}/libraries/AP_Scripting/lua_generated_bindings -i ${CMAKE_SOURCE_DIR}/libraries/AP_Scripting/generator/description/bindings.desc
	DEPENDS gen-bindings ${CMAKE_SOURCE_DIR}/libraries/AP_Scripting/generator/description/bindings.desc
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	COMMENT "Generating Lua bindings"
)


#------------------------------------------------

set(mavlink_INCLUDES
    /home/buzz2/ardupilot/build2/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0
)

add_library(mavlink INTERFACE ${mavlink_SOURCES}) #2
target_compile_definitions(mavlink INTERFACE -DLFS_NO_DEBUG -DLFS_NO_WARN -DLFS_NO_ERROR -DLFS_NO_ASSERT) #4
target_include_directories(mavlink INTERFACE ${mavlink_INCLUDES})
set_target_properties(mavlink PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
    LINKER_LANGUAGE CXX
)

#------------------------------------------------

set(canard_SOURCES #3
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard/canard.c
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard/canard.h
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard/canard_internals.h
)

set(canard_INCLUDES
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
)

add_library(canard STATIC ${canard_SOURCES}) #2
target_include_directories(canard PUBLIC ${canard_INCLUDES})
set_target_properties(canard PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
    LINKER_LANGUAGE CXX
)

#------------------------------------------------

set(dronecan_dsdlc_generated_SOURCES #3
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryCells.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryContinuous.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryInfoAux.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryPeriodic.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.power.BatteryTag.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.proximity_sensor.Proximity.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.equipment.trafficmonitor.TrafficReport.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.Heading.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.MovingBaselineData.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.RelPosHeading.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.gnss.Status.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.Button.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.NotifyState.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/ardupilot.indication.SafetyState.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hex.equipment.flow.Measurement.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.himark.servo.ServoCmd.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.himark.servo.ServoInfo.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetEscID.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetMaintenanceInformation_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetMaintenanceInformation_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetMajorConfig_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.GetMajorConfig_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.RawCommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SelfTest_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SelfTest_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetAngle_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetAngle_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetBaud_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetBaud_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetDirection_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetDirection_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetID_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetID_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetLED_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetLED_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetReportingFrequency_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetReportingFrequency_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetThrottleSource_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.SetThrottleSource_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.StatusMsg1.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.StatusMsg2.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.hobbywing.esc.StatusMsg3.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.FocCtrl.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.PUSHCAN.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.PUSHSCI.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.ParamCfg.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.tmotor.esc.ParamGet.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.volz.servo.ActuatorStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.CopterAttStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.GimbalAttitudeStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.GimbalControlData.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.GnssStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/com.xacti.GnssStatusReq.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/cuav.equipment.power.CBAT.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.protocol.CanStats.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.protocol.FlexDebug.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.protocol.GlobalTime.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.protocol.Stats.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.ArmStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.BasicID.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.Location.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.OperatorID.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.SecureCommand_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.SecureCommand_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.SelfID.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.remoteid.System.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.hygrometer.Hygrometer.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.rc.RCInput.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/dronecan.sensors.rpm.RPM.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.OutputEnable_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.OutputEnable_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/mppt.Stream.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.CoarseOrientation.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.Timestamp.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.ArrayCommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.Command.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.actuator.Status.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.MagneticFieldStrength.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.MagneticFieldStrength2.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.RawIMU.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ahrs.Solution.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.AngleOfAttack.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.IndicatedAirspeed.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.RawAirData.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.Sideslip.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.StaticPressure.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.StaticTemperature.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.air_data.TrueAirspeed.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.AngularCommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.GEOPOICommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.Mode.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.camera_gimbal.Status.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.device.Temperature.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.RPMCommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.RawCommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.Status.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.esc.StatusExtended.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Auxiliary.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.ECEFPositionVelocity.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Fix.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.Fix2.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.gnss.RTCMStream.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.hardpoint.Command.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.hardpoint.Status.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.FuelTankStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.reciprocating.CylinderStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.ice.reciprocating.Status.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.BeepCommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.LightsCommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.RGB565.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.indication.SingleLightCommand.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.BatteryInfo.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.CircuitStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.power.PrimaryPowerSupplyStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.range_sensor.Measurement.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.equipment.safety.ArmingStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.navigation.GlobalNavigationSolution.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.AccessCommandShell_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.AccessCommandShell_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.CANIfaceStats.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.DataTypeKind.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetDataTypeInfo_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetDataTypeInfo_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetNodeInfo_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetNodeInfo_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetTransportStats_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GetTransportStats_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.GlobalTimeSync.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.HardwareVersion.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.NodeStatus.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.Panic.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.RestartNode_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.RestartNode_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.SoftwareVersion.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.KeyValue.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.LogLevel.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.debug.LogMessage.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.Allocation.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.AppendEntries_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.AppendEntries_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.Discovery.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.Entry.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.RequestVote_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.dynamic_node_id.server.RequestVote_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Begin_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Begin_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.enumeration.Indication.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.BeginFirmwareUpdate_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Delete_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Delete_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.EntryType.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Error.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetDirectoryEntryInfo_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetDirectoryEntryInfo_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetInfo_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.GetInfo_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Path.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Read_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Read_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Write_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.file.Write_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.Empty.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.ExecuteOpcode_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.ExecuteOpcode_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.GetSet_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.GetSet_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.NumericValue.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.protocol.param.Value.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Broadcast.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Call_req.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Call_res.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Protocol.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.SerialConfig.c
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/src/uavcan.tunnel.Targetted.c
)

set(dronecan_dsdlc_generated_INCLUDES
    build2/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    modules/DroneCAN/libcanard
)

add_library(dronecan_dsdlc_generated STATIC ${dronecan_dsdlc_generated_SOURCES}) #2
target_include_directories(dronecan_dsdlc_generated PUBLIC ${dronecan_dsdlc_generated_INCLUDES})
set_target_properties(dronecan_dsdlc_generated PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
    LINKER_LANGUAGE C
)

#------------------------------------------------

set(littlefs_SOURCES #3
	modules/littlefs/lfs.h
	modules/littlefs/lfs_util.h
	modules/littlefs/lfs.c
	modules/littlefs/lfs_util.c
	modules/littlefs/bd/lfs_emubd.h
	modules/littlefs/bd/lfs_filebd.h
	modules/littlefs/bd/lfs_rambd.h
	modules/littlefs/bd/lfs_filebd.c
)

set(littlefs_INCLUDES
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include)

add_library(littlefs ${littlefs_SOURCES}) #2
target_compile_definitions(littlefs PUBLIC -DLFS_NO_DEBUG -DLFS_NO_WARN -DLFS_NO_ERROR -DLFS_NO_ASSERT) #1
target_include_directories(littlefs PUBLIC ${littlefs_INCLUDES})

set_target_properties(littlefs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)

#------------------------------------------------

set(AC_AttitudeControl_SOURCES #3
	libraries/AC_AttitudeControl/AC_AttitudeControl.h
	libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.h
	libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h
	libraries/AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h
	libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.h
	libraries/AC_AttitudeControl/AC_AttitudeControl_TS.h
	libraries/AC_AttitudeControl/AC_CommandModel.h
	libraries/AC_AttitudeControl/AC_PosControl.h
	libraries/AC_AttitudeControl/AC_WeatherVane.h
	libraries/AC_AttitudeControl/LogStructure.h
	libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
	libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp
	libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp
	libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
	libraries/AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.cpp
	libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.cpp
	libraries/AC_AttitudeControl/AC_AttitudeControl_TS.cpp
	libraries/AC_AttitudeControl/AC_CommandModel.cpp
	libraries/AC_AttitudeControl/AC_PosControl.cpp
	libraries/AC_AttitudeControl/AC_PosControl_Logging.cpp
	libraries/AC_AttitudeControl/AC_WeatherVane.cpp
)

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
target_include_directories(AC_AttitudeControl PUBLIC ${AC_AttitudeControl_INCLUDES})

set_target_properties(AC_AttitudeControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_AttitudeControl mavlink) #clean
target_link_libraries(AC_AttitudeControl canard) #clean
target_link_libraries(AC_AttitudeControl littlefs) #clean

#------------------------------------------------

set(AC_AutoTune_SOURCES #3
	libraries/AC_AutoTune/AC_AutoTune.h
	libraries/AC_AutoTune/AC_AutoTune_FreqResp.h
	libraries/AC_AutoTune/AC_AutoTune_Heli.h
	libraries/AC_AutoTune/AC_AutoTune_Multi.h
	libraries/AC_AutoTune/AC_AutoTune_config.h
	libraries/AC_AutoTune/AC_AutoTune.cpp
	libraries/AC_AutoTune/AC_AutoTune_FreqResp.cpp
	libraries/AC_AutoTune/AC_AutoTune_Heli.cpp
	libraries/AC_AutoTune/AC_AutoTune_Multi.cpp
)

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
target_include_directories(AC_AutoTune PUBLIC ${AC_AutoTune_INCLUDES})

set_target_properties(AC_AutoTune PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_AutoTune mavlink) #clean
target_link_libraries(AC_AutoTune canard) #clean
target_link_libraries(AC_AutoTune littlefs) #clean

#------------------------------------------------

set(AC_Autorotation_SOURCES #3
	libraries/AC_Autorotation/AC_Autorotation.h
	libraries/AC_Autorotation/RSC_Autorotation.h
	libraries/AC_Autorotation/AC_Autorotation.cpp
	libraries/AC_Autorotation/RSC_Autorotation.cpp
)

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
target_include_directories(AC_Autorotation PUBLIC ${AC_Autorotation_INCLUDES})

set_target_properties(AC_Autorotation PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_Autorotation mavlink) #clean
target_link_libraries(AC_Autorotation canard) #clean
target_link_libraries(AC_Autorotation littlefs) #clean

#------------------------------------------------

set(AC_Avoidance_SOURCES #3
	libraries/AC_Avoidance/AC_Avoid.h
	libraries/AC_Avoidance/AC_Avoidance_config.h
	libraries/AC_Avoidance/AP_OABendyRuler.h
	libraries/AC_Avoidance/AP_OADatabase.h
	libraries/AC_Avoidance/AP_OADijkstra.h
	libraries/AC_Avoidance/AP_OAPathPlanner.h
	libraries/AC_Avoidance/AP_OAVisGraph.h
	libraries/AC_Avoidance/LogStructure.h
	libraries/AC_Avoidance/AC_Avoid.cpp
	libraries/AC_Avoidance/AC_Avoidance_Logging.cpp
	libraries/AC_Avoidance/AP_OABendyRuler.cpp
	libraries/AC_Avoidance/AP_OADatabase.cpp
	libraries/AC_Avoidance/AP_OADijkstra.cpp
	libraries/AC_Avoidance/AP_OAPathPlanner.cpp
	libraries/AC_Avoidance/AP_OAVisGraph.cpp
)

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
target_include_directories(AC_Avoidance PUBLIC ${AC_Avoidance_INCLUDES})

set_target_properties(AC_Avoidance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_Avoidance mavlink) #clean
target_link_libraries(AC_Avoidance canard) #clean
target_link_libraries(AC_Avoidance littlefs) #clean

#------------------------------------------------

set(AC_CustomControl_SOURCES #3
	libraries/AC_CustomControl/AC_CustomControl.h
	libraries/AC_CustomControl/AC_CustomControl_Backend.h
	libraries/AC_CustomControl/AC_CustomControl_Empty.h
	libraries/AC_CustomControl/AC_CustomControl_PID.h
	libraries/AC_CustomControl/AC_CustomControl_config.h
	libraries/AC_CustomControl/AC_CustomControl.cpp
	libraries/AC_CustomControl/AC_CustomControl_Empty.cpp
	libraries/AC_CustomControl/AC_CustomControl_PID.cpp
)

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
target_include_directories(AC_CustomControl PUBLIC ${AC_CustomControl_INCLUDES})

set_target_properties(AC_CustomControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_CustomControl mavlink) #clean
target_link_libraries(AC_CustomControl canard) #clean
target_link_libraries(AC_CustomControl littlefs) #clean

#------------------------------------------------

set(AC_InputManager_SOURCES #3
	libraries/AC_InputManager/AC_InputManager.h
	libraries/AC_InputManager/AC_InputManager_Heli.h
	libraries/AC_InputManager/AC_InputManager.cpp
	libraries/AC_InputManager/AC_InputManager_Heli.cpp
)

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
target_include_directories(AC_InputManager PUBLIC ${AC_InputManager_INCLUDES})

set_target_properties(AC_InputManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_InputManager mavlink) #clean
target_link_libraries(AC_InputManager canard) #clean
target_link_libraries(AC_InputManager littlefs) #clean

#------------------------------------------------

set(AC_PID_SOURCES #3
	libraries/AC_PID/AC_HELI_PID.h
	libraries/AC_PID/AC_P.h
	libraries/AC_PID/AC_PI.h
	libraries/AC_PID/AC_PID.h
	libraries/AC_PID/AC_PID_2D.h
	libraries/AC_PID/AC_PID_Basic.h
	libraries/AC_PID/AC_PI_2D.h
	libraries/AC_PID/AC_P_1D.h
	libraries/AC_PID/AC_P_2D.h
	libraries/AC_PID/AP_PIDInfo.h
	libraries/AC_PID/AC_HELI_PID.cpp
	libraries/AC_PID/AC_P.cpp
	libraries/AC_PID/AC_PI.cpp
	libraries/AC_PID/AC_PID.cpp
	libraries/AC_PID/AC_PID_2D.cpp
	libraries/AC_PID/AC_PID_Basic.cpp
	libraries/AC_PID/AC_PI_2D.cpp
	libraries/AC_PID/AC_P_1D.cpp
	libraries/AC_PID/AC_P_2D.cpp
)

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
target_include_directories(AC_PID PUBLIC ${AC_PID_INCLUDES})

set_target_properties(AC_PID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_PID mavlink) #clean
target_link_libraries(AC_PID canard) #clean
target_link_libraries(AC_PID littlefs) #clean

#------------------------------------------------

set(AC_PrecLand_SOURCES #3
	libraries/AC_PrecLand/AC_PrecLand.h
	libraries/AC_PrecLand/AC_PrecLand_Backend.h
	libraries/AC_PrecLand/AC_PrecLand_IRLock.h
	libraries/AC_PrecLand/AC_PrecLand_MAVLink.h
	libraries/AC_PrecLand/AC_PrecLand_SITL.h
	libraries/AC_PrecLand/AC_PrecLand_SITL_Gazebo.h
	libraries/AC_PrecLand/AC_PrecLand_StateMachine.h
	libraries/AC_PrecLand/AC_PrecLand_config.h
	libraries/AC_PrecLand/LogStructure.h
	libraries/AC_PrecLand/PosVelEKF.h
	libraries/AC_PrecLand/AC_PrecLand.cpp
	libraries/AC_PrecLand/AC_PrecLand_IRLock.cpp
	libraries/AC_PrecLand/AC_PrecLand_MAVLink.cpp
	libraries/AC_PrecLand/AC_PrecLand_SITL.cpp
	libraries/AC_PrecLand/AC_PrecLand_SITL_Gazebo.cpp
	libraries/AC_PrecLand/AC_PrecLand_StateMachine.cpp
	libraries/AC_PrecLand/PosVelEKF.cpp
)

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
target_include_directories(AC_PrecLand PUBLIC ${AC_PrecLand_INCLUDES})

set_target_properties(AC_PrecLand PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_PrecLand mavlink) #clean
target_link_libraries(AC_PrecLand canard) #clean
target_link_libraries(AC_PrecLand littlefs) #clean

#------------------------------------------------

set(AC_Sprayer_SOURCES #3
	libraries/AC_Sprayer/AC_Sprayer.h
	libraries/AC_Sprayer/AC_Sprayer_config.h
	libraries/AC_Sprayer/AC_Sprayer.cpp
)

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
target_include_directories(AC_Sprayer PUBLIC ${AC_Sprayer_INCLUDES})

set_target_properties(AC_Sprayer PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_Sprayer mavlink) #clean
target_link_libraries(AC_Sprayer canard) #clean
target_link_libraries(AC_Sprayer littlefs) #clean

#------------------------------------------------

set(AC_WPNav_SOURCES #3
	libraries/AC_WPNav/AC_Circle.h
	libraries/AC_WPNav/AC_Loiter.h
	libraries/AC_WPNav/AC_WPNav.h
	libraries/AC_WPNav/AC_WPNav_OA.h
	libraries/AC_WPNav/AC_WPNav_config.h
	libraries/AC_WPNav/AC_Circle.cpp
	libraries/AC_WPNav/AC_Loiter.cpp
	libraries/AC_WPNav/AC_WPNav.cpp
	libraries/AC_WPNav/AC_WPNav_OA.cpp
)

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
target_include_directories(AC_WPNav PUBLIC ${AC_WPNav_INCLUDES})

set_target_properties(AC_WPNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_WPNav mavlink) #clean
target_link_libraries(AC_WPNav canard) #clean
target_link_libraries(AC_WPNav littlefs) #clean

#------------------------------------------------

set(APM_Control_SOURCES #3
	libraries/APM_Control/APM_Control.h
	libraries/APM_Control/AP_AutoTune.h
	libraries/APM_Control/AP_FW_Controller.h
	libraries/APM_Control/AP_PitchController.h
	libraries/APM_Control/AP_RollController.h
	libraries/APM_Control/AP_SteerController.h
	libraries/APM_Control/AP_YawController.h
	libraries/APM_Control/AR_AttitudeControl.h
	libraries/APM_Control/AR_PosControl.h
	libraries/APM_Control/AP_AutoTune.cpp
	libraries/APM_Control/AP_FW_Controller.cpp
	libraries/APM_Control/AP_PitchController.cpp
	libraries/APM_Control/AP_RollController.cpp
	libraries/APM_Control/AP_SteerController.cpp
	libraries/APM_Control/AP_YawController.cpp
	libraries/APM_Control/AR_AttitudeControl.cpp
	libraries/APM_Control/AR_PosControl.cpp
)

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
target_include_directories(APM_Control PUBLIC ${APM_Control_INCLUDES})

set_target_properties(APM_Control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(APM_Control mavlink) #clean
target_link_libraries(APM_Control canard) #clean
target_link_libraries(APM_Control littlefs) #clean

#------------------------------------------------

set(AP_ADC_SOURCES #3
	libraries/AP_ADC/AP_ADC_ADS1115.h
	libraries/AP_ADC/AP_ADC_ADS1115.cpp
)

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
target_include_directories(AP_ADC PUBLIC ${AP_ADC_INCLUDES})

set_target_properties(AP_ADC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ADC mavlink) #clean
target_link_libraries(AP_ADC canard) #clean
target_link_libraries(AP_ADC littlefs) #clean

#------------------------------------------------

set(AP_ADSB_SOURCES #3
	libraries/AP_ADSB/AP_ADSB.h
	libraries/AP_ADSB/AP_ADSB_Backend.h
	libraries/AP_ADSB/AP_ADSB_Sagetech.h
	libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.h
	libraries/AP_ADSB/AP_ADSB_config.h
	libraries/AP_ADSB/AP_ADSB_uAvionix_MAVLink.h
	libraries/AP_ADSB/AP_ADSB_uAvionix_UCP.h
	libraries/AP_ADSB/AP_ADSB.cpp
	libraries/AP_ADSB/AP_ADSB_Backend.cpp
	libraries/AP_ADSB/AP_ADSB_Sagetech.cpp
	libraries/AP_ADSB/AP_ADSB_Sagetech_MXS.cpp
	libraries/AP_ADSB/AP_ADSB_uAvionix_MAVLink.cpp
	libraries/AP_ADSB/AP_ADSB_uAvionix_UCP.cpp
)

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
target_include_directories(AP_ADSB PUBLIC ${AP_ADSB_INCLUDES})

set_target_properties(AP_ADSB PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ADSB mavlink) #clean
target_link_libraries(AP_ADSB canard) #clean
target_link_libraries(AP_ADSB littlefs) #clean

#------------------------------------------------

set(AP_AHRS_SOURCES #3
	libraries/AP_AHRS/AP_AHRS.h
	libraries/AP_AHRS/AP_AHRS_Backend.h
	libraries/AP_AHRS/AP_AHRS_DCM.h
	libraries/AP_AHRS/AP_AHRS_External.h
	libraries/AP_AHRS/AP_AHRS_SIM.h
	libraries/AP_AHRS/AP_AHRS_View.h
	libraries/AP_AHRS/AP_AHRS_config.h
	libraries/AP_AHRS/LogStructure.h
	libraries/AP_AHRS/AP_AHRS.cpp
	libraries/AP_AHRS/AP_AHRS_Backend.cpp
	libraries/AP_AHRS/AP_AHRS_DCM.cpp
	libraries/AP_AHRS/AP_AHRS_External.cpp
	libraries/AP_AHRS/AP_AHRS_Logging.cpp
	libraries/AP_AHRS/AP_AHRS_SIM.cpp
	libraries/AP_AHRS/AP_AHRS_View.cpp
)

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
target_include_directories(AP_AHRS PUBLIC ${AP_AHRS_INCLUDES})

set_target_properties(AP_AHRS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_AHRS mavlink) #clean
target_link_libraries(AP_AHRS canard) #clean
target_link_libraries(AP_AHRS littlefs) #clean

#------------------------------------------------

set(AP_AccelCal_SOURCES #3
	libraries/AP_AccelCal/AP_AccelCal.h
	libraries/AP_AccelCal/AccelCalibrator.h
	libraries/AP_AccelCal/AP_AccelCal.cpp
	libraries/AP_AccelCal/AccelCalibrator.cpp
)

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
target_include_directories(AP_AccelCal PUBLIC ${AP_AccelCal_INCLUDES})

set_target_properties(AP_AccelCal PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_AccelCal mavlink) #clean
target_link_libraries(AP_AccelCal canard) #clean
target_link_libraries(AP_AccelCal littlefs) #clean

#------------------------------------------------

set(AP_AdvancedFailsafe_SOURCES #3
	libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h
	libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe_config.h
	libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
)

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
target_include_directories(AP_AdvancedFailsafe PUBLIC ${AP_AdvancedFailsafe_INCLUDES})

set_target_properties(AP_AdvancedFailsafe PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_AdvancedFailsafe mavlink) #clean
target_link_libraries(AP_AdvancedFailsafe canard) #clean
target_link_libraries(AP_AdvancedFailsafe littlefs) #clean

#------------------------------------------------

set(AP_Airspeed_SOURCES #3
	libraries/AP_Airspeed/AP_Airspeed.h
	libraries/AP_Airspeed/AP_Airspeed_ASP5033.h
	libraries/AP_Airspeed/AP_Airspeed_AUAV.h
	libraries/AP_Airspeed/AP_Airspeed_Backend.h
	libraries/AP_Airspeed/AP_Airspeed_DLVR.h
	libraries/AP_Airspeed/AP_Airspeed_DroneCAN.h
	libraries/AP_Airspeed/AP_Airspeed_External.h
	libraries/AP_Airspeed/AP_Airspeed_MS4525.h
	libraries/AP_Airspeed/AP_Airspeed_MS5525.h
	libraries/AP_Airspeed/AP_Airspeed_MSP.h
	libraries/AP_Airspeed/AP_Airspeed_NMEA.h
	libraries/AP_Airspeed/AP_Airspeed_SDP3X.h
	libraries/AP_Airspeed/AP_Airspeed_SITL.h
	libraries/AP_Airspeed/AP_Airspeed_analog.h
	libraries/AP_Airspeed/AP_Airspeed_config.h
	libraries/AP_Airspeed/AP_Airspeed.cpp
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
	libraries/AP_Airspeed/AP_Airspeed_NMEA.cpp
	libraries/AP_Airspeed/AP_Airspeed_Params.cpp
	libraries/AP_Airspeed/AP_Airspeed_SDP3X.cpp
	libraries/AP_Airspeed/AP_Airspeed_SITL.cpp
	libraries/AP_Airspeed/AP_Airspeed_analog.cpp
	libraries/AP_Airspeed/Airspeed_Calibration.cpp
)

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
target_include_directories(AP_Airspeed PUBLIC ${AP_Airspeed_INCLUDES})

set_target_properties(AP_Airspeed PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Airspeed mavlink) #clean
target_link_libraries(AP_Airspeed canard) #clean
target_link_libraries(AP_Airspeed littlefs) #clean

#------------------------------------------------

set(AP_Baro_SOURCES #3
	libraries/AP_Baro/AP_Baro.h
	libraries/AP_Baro/AP_Baro_AUAV.h
	libraries/AP_Baro/AP_Baro_BMP085.h
	libraries/AP_Baro/AP_Baro_BMP280.h
	libraries/AP_Baro/AP_Baro_BMP388.h
	libraries/AP_Baro/AP_Baro_BMP581.h
	libraries/AP_Baro/AP_Baro_Backend.h
	libraries/AP_Baro/AP_Baro_DPS280.h
	libraries/AP_Baro/AP_Baro_DroneCAN.h
	libraries/AP_Baro/AP_Baro_Dummy.h
	libraries/AP_Baro/AP_Baro_ExternalAHRS.h
	libraries/AP_Baro/AP_Baro_FBM320.h
	libraries/AP_Baro/AP_Baro_ICM20789.h
	libraries/AP_Baro/AP_Baro_ICP101XX.h
	libraries/AP_Baro/AP_Baro_ICP201XX.h
	libraries/AP_Baro/AP_Baro_KellerLD.h
	libraries/AP_Baro/AP_Baro_LPS2XH.h
	libraries/AP_Baro/AP_Baro_MS5611.h
	libraries/AP_Baro/AP_Baro_MSP.h
	libraries/AP_Baro/AP_Baro_SITL.h
	libraries/AP_Baro/AP_Baro_SPL06.h
	libraries/AP_Baro/AP_Baro_config.h
	libraries/AP_Baro/LogStructure.h
	libraries/AP_Baro/AP_Baro.cpp
	libraries/AP_Baro/AP_Baro_AUAV.cpp
	libraries/AP_Baro/AP_Baro_BMP085.cpp
	libraries/AP_Baro/AP_Baro_BMP280.cpp
	libraries/AP_Baro/AP_Baro_BMP388.cpp
	libraries/AP_Baro/AP_Baro_BMP581.cpp
	libraries/AP_Baro/AP_Baro_Backend.cpp
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
	libraries/AP_Baro/AP_Baro_SITL.cpp
	libraries/AP_Baro/AP_Baro_SPL06.cpp
	libraries/AP_Baro/AP_Baro_Wind.cpp
	libraries/AP_Baro/AP_Baro_atmosphere.cpp
)

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
target_include_directories(AP_Baro PUBLIC ${AP_Baro_INCLUDES})

set_target_properties(AP_Baro PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Baro mavlink) #clean
target_link_libraries(AP_Baro canard) #clean
target_link_libraries(AP_Baro littlefs) #clean

#------------------------------------------------

set(AP_BattMonitor_SOURCES #3
	libraries/AP_BattMonitor/AP_BattMonitor.h
	libraries/AP_BattMonitor/AP_BattMonitor_AD7091R5.h
	libraries/AP_BattMonitor/AP_BattMonitor_Analog.h
	libraries/AP_BattMonitor/AP_BattMonitor_Backend.h
	libraries/AP_BattMonitor/AP_BattMonitor_Bebop.h
	libraries/AP_BattMonitor/AP_BattMonitor_DroneCAN.h
	libraries/AP_BattMonitor/AP_BattMonitor_EFI.h
	libraries/AP_BattMonitor/AP_BattMonitor_ESC.h
	libraries/AP_BattMonitor/AP_BattMonitor_FuelFlow.h
	libraries/AP_BattMonitor/AP_BattMonitor_FuelLevel_Analog.h
	libraries/AP_BattMonitor/AP_BattMonitor_FuelLevel_PWM.h
	libraries/AP_BattMonitor/AP_BattMonitor_Generator.h
	libraries/AP_BattMonitor/AP_BattMonitor_INA239.h
	libraries/AP_BattMonitor/AP_BattMonitor_INA2xx.h
	libraries/AP_BattMonitor/AP_BattMonitor_INA3221.h
	libraries/AP_BattMonitor/AP_BattMonitor_LTC2946.h
	libraries/AP_BattMonitor/AP_BattMonitor_Params.h
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus.h
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Generic.h
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Maxell.h
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_NeoDesign.h
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Rotoye.h
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_SUI.h
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Solo.h
	libraries/AP_BattMonitor/AP_BattMonitor_Scripting.h
	libraries/AP_BattMonitor/AP_BattMonitor_Sum.h
	libraries/AP_BattMonitor/AP_BattMonitor_Synthetic_Current.h
	libraries/AP_BattMonitor/AP_BattMonitor_Torqeedo.h
	libraries/AP_BattMonitor/AP_BattMonitor_config.h
	libraries/AP_BattMonitor/LogStructure.h
	libraries/AP_BattMonitor/AP_BattMonitor.cpp
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
	libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Generic.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_NeoDesign.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Rotoye.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_SUI.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_SMBus_Solo.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_Scripting.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_Sum.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_Synthetic_Current.cpp
	libraries/AP_BattMonitor/AP_BattMonitor_Torqeedo.cpp
)

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
target_include_directories(AP_BattMonitor PUBLIC ${AP_BattMonitor_INCLUDES})

set_target_properties(AP_BattMonitor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_BattMonitor mavlink) #clean
target_link_libraries(AP_BattMonitor canard) #clean
target_link_libraries(AP_BattMonitor littlefs) #clean

#------------------------------------------------

set(AP_Beacon_SOURCES #3
	libraries/AP_Beacon/AP_Beacon.h
	libraries/AP_Beacon/AP_Beacon_Backend.h
	libraries/AP_Beacon/AP_Beacon_Marvelmind.h
	libraries/AP_Beacon/AP_Beacon_Nooploop.h
	libraries/AP_Beacon/AP_Beacon_Pozyx.h
	libraries/AP_Beacon/AP_Beacon_SITL.h
	libraries/AP_Beacon/AP_Beacon_config.h
	libraries/AP_Beacon/LogStructure.h
	libraries/AP_Beacon/AP_Beacon.cpp
	libraries/AP_Beacon/AP_Beacon_Backend.cpp
	libraries/AP_Beacon/AP_Beacon_Marvelmind.cpp
	libraries/AP_Beacon/AP_Beacon_Nooploop.cpp
	libraries/AP_Beacon/AP_Beacon_Pozyx.cpp
	libraries/AP_Beacon/AP_Beacon_SITL.cpp
)

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
target_include_directories(AP_Beacon PUBLIC ${AP_Beacon_INCLUDES})

set_target_properties(AP_Beacon PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Beacon mavlink) #clean
target_link_libraries(AP_Beacon canard) #clean
target_link_libraries(AP_Beacon littlefs) #clean

#------------------------------------------------

set(AP_BoardConfig_SOURCES #3
	libraries/AP_BoardConfig/AP_BoardConfig.h
	libraries/AP_BoardConfig/AP_BoardConfig_config.h
	libraries/AP_BoardConfig/AP_BoardConfig.cpp
	libraries/AP_BoardConfig/IMU_heater.cpp
	libraries/AP_BoardConfig/board_drivers.cpp
)

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
target_include_directories(AP_BoardConfig PUBLIC ${AP_BoardConfig_INCLUDES})

set_target_properties(AP_BoardConfig PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_BoardConfig mavlink) #clean
target_link_libraries(AP_BoardConfig canard) #clean
target_link_libraries(AP_BoardConfig littlefs) #clean

#------------------------------------------------

set(AP_Button_SOURCES #3
	libraries/AP_Button/AP_Button.h
	libraries/AP_Button/AP_Button.cpp
)

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
target_include_directories(AP_Button PUBLIC ${AP_Button_INCLUDES})

set_target_properties(AP_Button PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Button mavlink) #clean
target_link_libraries(AP_Button canard) #clean
target_link_libraries(AP_Button littlefs) #clean

#------------------------------------------------

set(AP_CANManager_SOURCES #3
	libraries/AP_CANManager/AP_CAN.h
	libraries/AP_CANManager/AP_CANDriver.h
	libraries/AP_CANManager/AP_CANManager.h
	libraries/AP_CANManager/AP_CANManager_config.h
	libraries/AP_CANManager/AP_CANSensor.h
	libraries/AP_CANManager/AP_MAVLinkCAN.h
	libraries/AP_CANManager/AP_SLCANIface.h
	libraries/AP_CANManager/LogStructure.h
	libraries/AP_CANManager/AP_CANIfaceParams.cpp
	libraries/AP_CANManager/AP_CANManager.cpp
	libraries/AP_CANManager/AP_CANManager_CANDriver_Params.cpp
	libraries/AP_CANManager/AP_CANSensor.cpp
	libraries/AP_CANManager/AP_MAVLinkCAN.cpp
	libraries/AP_CANManager/AP_SLCANIface.cpp
)

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
target_include_directories(AP_CANManager PUBLIC ${AP_CANManager_INCLUDES})

set_target_properties(AP_CANManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_CANManager mavlink) #clean
target_link_libraries(AP_CANManager canard) #clean
target_link_libraries(AP_CANManager littlefs) #clean

#------------------------------------------------

set(AP_CSVReader_SOURCES #3
	libraries/AP_CSVReader/AP_CSVReader.h
	libraries/AP_CSVReader/AP_CSVReader.cpp
)

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
target_include_directories(AP_CSVReader PUBLIC ${AP_CSVReader_INCLUDES})

set_target_properties(AP_CSVReader PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_CSVReader mavlink) #clean
target_link_libraries(AP_CSVReader canard) #clean
target_link_libraries(AP_CSVReader littlefs) #clean

#------------------------------------------------

set(AP_Camera_SOURCES #3
	libraries/AP_Camera/AP_Camera.h
	libraries/AP_Camera/AP_Camera_Backend.h
	libraries/AP_Camera/AP_Camera_MAVLink.h
	libraries/AP_Camera/AP_Camera_MAVLinkCamV2.h
	libraries/AP_Camera/AP_Camera_Mount.h
	libraries/AP_Camera/AP_Camera_Params.h
	libraries/AP_Camera/AP_Camera_Relay.h
	libraries/AP_Camera/AP_Camera_Scripting.h
	libraries/AP_Camera/AP_Camera_Servo.h
	libraries/AP_Camera/AP_Camera_SoloGimbal.h
	libraries/AP_Camera/AP_Camera_config.h
	libraries/AP_Camera/AP_Camera_shareddefs.h
	libraries/AP_Camera/AP_RunCam.h
	libraries/AP_Camera/LogStructure.h
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
	libraries/AP_Camera/AP_RunCam.cpp
)

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
target_include_directories(AP_Camera PUBLIC ${AP_Camera_INCLUDES})

set_target_properties(AP_Camera PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Camera mavlink) #clean
target_link_libraries(AP_Camera canard) #clean
target_link_libraries(AP_Camera littlefs) #clean

#------------------------------------------------

set(AP_CheckFirmware_SOURCES #3
	libraries/AP_CheckFirmware/AP_CheckFirmware.h
	libraries/AP_CheckFirmware/AP_CheckFirmwareDefine.h
	libraries/AP_CheckFirmware/monocypher.h
	libraries/AP_CheckFirmware/AP_CheckFirmware.cpp
	libraries/AP_CheckFirmware/AP_CheckFirmware_secure_command.cpp
	libraries/AP_CheckFirmware/monocypher.cpp
)

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
target_include_directories(AP_CheckFirmware PUBLIC ${AP_CheckFirmware_INCLUDES})

set_target_properties(AP_CheckFirmware PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_CheckFirmware mavlink) #clean
target_link_libraries(AP_CheckFirmware canard) #clean
target_link_libraries(AP_CheckFirmware littlefs) #clean

#------------------------------------------------

set(AP_Common_SOURCES #3
	libraries/AP_Common/AP_Common.h
	libraries/AP_Common/AP_ExpandingArray.h
	libraries/AP_Common/AP_FWVersion.h
	libraries/AP_Common/AP_FWVersionDefine.h
	libraries/AP_Common/AP_Test.h
	libraries/AP_Common/Bitmask.h
	libraries/AP_Common/ExpandingString.h
	libraries/AP_Common/Location.h
	libraries/AP_Common/NMEA.h
	libraries/AP_Common/TSIndex.h
	libraries/AP_Common/float16.h
	libraries/AP_Common/sorting.h
	libraries/AP_Common/time.h
	libraries/AP_Common/AP_Common.cpp
	libraries/AP_Common/AP_ExpandingArray.cpp
	libraries/AP_Common/AP_FWVersion.cpp
	libraries/AP_Common/ExpandingString.cpp
	libraries/AP_Common/Location.cpp
	libraries/AP_Common/NMEA.cpp
	libraries/AP_Common/c++.cpp
	libraries/AP_Common/float16.cpp
	libraries/AP_Common/sorting.cpp
	libraries/AP_Common/time.cpp
)

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
target_include_directories(AP_Common PUBLIC ${AP_Common_INCLUDES})

set_target_properties(AP_Common PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Common mavlink) #clean
target_link_libraries(AP_Common canard) #clean
target_link_libraries(AP_Common littlefs) #clean

#------------------------------------------------

set(AP_Compass_SOURCES #3
	libraries/AP_Compass/AP_Compass.h
	libraries/AP_Compass/AP_Compass_AK09916.h
	libraries/AP_Compass/AP_Compass_AK8963.h
	libraries/AP_Compass/AP_Compass_BMM150.h
	libraries/AP_Compass/AP_Compass_BMM350.h
	libraries/AP_Compass/AP_Compass_Backend.h
	libraries/AP_Compass/AP_Compass_DroneCAN.h
	libraries/AP_Compass/AP_Compass_ExternalAHRS.h
	libraries/AP_Compass/AP_Compass_HMC5843.h
	libraries/AP_Compass/AP_Compass_IIS2MDC.h
	libraries/AP_Compass/AP_Compass_IST8308.h
	libraries/AP_Compass/AP_Compass_IST8310.h
	libraries/AP_Compass/AP_Compass_LIS2MDL.h
	libraries/AP_Compass/AP_Compass_LIS3MDL.h
	libraries/AP_Compass/AP_Compass_LSM303D.h
	libraries/AP_Compass/AP_Compass_LSM9DS1.h
	libraries/AP_Compass/AP_Compass_MAG3110.h
	libraries/AP_Compass/AP_Compass_MMC3416.h
	libraries/AP_Compass/AP_Compass_MMC5xx3.h
	libraries/AP_Compass/AP_Compass_MSP.h
	libraries/AP_Compass/AP_Compass_QMC5883L.h
	libraries/AP_Compass/AP_Compass_QMC5883P.h
	libraries/AP_Compass/AP_Compass_RM3100.h
	libraries/AP_Compass/AP_Compass_SITL.h
	libraries/AP_Compass/AP_Compass_config.h
	libraries/AP_Compass/CompassCalibrator.h
	libraries/AP_Compass/Compass_PerMotor.h
	libraries/AP_Compass/Compass_learn.h
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
	libraries/AP_Compass/Compass_learn.cpp
)

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
target_include_directories(AP_Compass PUBLIC ${AP_Compass_INCLUDES})

set_target_properties(AP_Compass PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Compass mavlink) #clean
target_link_libraries(AP_Compass canard) #clean
target_link_libraries(AP_Compass littlefs) #clean

#------------------------------------------------

set(AP_CustomRotations_SOURCES #3
	libraries/AP_CustomRotations/AP_CustomRotations.h
	libraries/AP_CustomRotations/AP_CustomRotations_config.h
	libraries/AP_CustomRotations/AP_CustomRotations.cpp
	libraries/AP_CustomRotations/AP_CustomRotations_params.cpp
)

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
target_include_directories(AP_CustomRotations PUBLIC ${AP_CustomRotations_INCLUDES})

set_target_properties(AP_CustomRotations PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_CustomRotations mavlink) #clean
target_link_libraries(AP_CustomRotations canard) #clean
target_link_libraries(AP_CustomRotations littlefs) #clean

#------------------------------------------------

set(AP_DAL_SOURCES #3
	libraries/AP_DAL/AP_DAL.h
	libraries/AP_DAL/AP_DAL_Airspeed.h
	libraries/AP_DAL/AP_DAL_Baro.h
	libraries/AP_DAL/AP_DAL_Beacon.h
	libraries/AP_DAL/AP_DAL_Compass.h
	libraries/AP_DAL/AP_DAL_GPS.h
	libraries/AP_DAL/AP_DAL_InertialSensor.h
	libraries/AP_DAL/AP_DAL_RangeFinder.h
	libraries/AP_DAL/AP_DAL_VisualOdom.h
	libraries/AP_DAL/LogStructure.h
	libraries/AP_DAL/AP_DAL.cpp
	libraries/AP_DAL/AP_DAL_Airspeed.cpp
	libraries/AP_DAL/AP_DAL_Baro.cpp
	libraries/AP_DAL/AP_DAL_Beacon.cpp
	libraries/AP_DAL/AP_DAL_Compass.cpp
	libraries/AP_DAL/AP_DAL_GPS.cpp
	libraries/AP_DAL/AP_DAL_InertialSensor.cpp
	libraries/AP_DAL/AP_DAL_RangeFinder.cpp
	libraries/AP_DAL/AP_DAL_VisualOdom.cpp
)

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
target_include_directories(AP_DAL PUBLIC ${AP_DAL_INCLUDES})

set_target_properties(AP_DAL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_DAL mavlink) #clean
target_link_libraries(AP_DAL canard) #clean
target_link_libraries(AP_DAL littlefs) #clean

#------------------------------------------------

set(AP_DDS_SOURCES #3
	libraries/AP_DDS/AP_DDS_Client.h
	libraries/AP_DDS/AP_DDS_ExternalControl.h
	libraries/AP_DDS/AP_DDS_External_Odom.h
	libraries/AP_DDS/AP_DDS_Frames.h
	libraries/AP_DDS/AP_DDS_Service_Table.h
	libraries/AP_DDS/AP_DDS_Topic_Table.h
	libraries/AP_DDS/AP_DDS_Type_Conversions.h
	libraries/AP_DDS/AP_DDS_config.h
	libraries/AP_DDS/AP_DDS_Client.cpp
	libraries/AP_DDS/AP_DDS_ExternalControl.cpp
	libraries/AP_DDS/AP_DDS_External_Odom.cpp
	libraries/AP_DDS/AP_DDS_Serial.cpp
	libraries/AP_DDS/AP_DDS_Type_Conversions.cpp
	libraries/AP_DDS/AP_DDS_UDP.cpp
)

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
target_include_directories(AP_DDS PUBLIC ${AP_DDS_INCLUDES})

set_target_properties(AP_DDS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_DDS mavlink) #clean
target_link_libraries(AP_DDS canard) #clean
target_link_libraries(AP_DDS littlefs) #clean

#------------------------------------------------

set(AP_Declination_SOURCES #3
	libraries/AP_Declination/AP_Declination.h
	libraries/AP_Declination/AP_Declination.cpp
	libraries/AP_Declination/tables.cpp
)

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
target_include_directories(AP_Declination PUBLIC ${AP_Declination_INCLUDES})

set_target_properties(AP_Declination PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Declination mavlink) #clean
target_link_libraries(AP_Declination canard) #clean
target_link_libraries(AP_Declination littlefs) #clean

#------------------------------------------------

set(AP_Devo_Telem_SOURCES #3
	libraries/AP_Devo_Telem/AP_Devo_Telem.h
	libraries/AP_Devo_Telem/AP_Devo_Telem.cpp
)

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
target_include_directories(AP_Devo_Telem PUBLIC ${AP_Devo_Telem_INCLUDES})

set_target_properties(AP_Devo_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Devo_Telem mavlink) #clean
target_link_libraries(AP_Devo_Telem canard) #clean
target_link_libraries(AP_Devo_Telem littlefs) #clean

#------------------------------------------------

set(AP_DroneCAN_SOURCES #3
	libraries/AP_DroneCAN/AP_Canard_iface.h
	libraries/AP_DroneCAN/AP_DroneCAN.h
	libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.h
	libraries/AP_DroneCAN/AP_DroneCAN_serial.h
	libraries/AP_DroneCAN/AP_Canard_iface.cpp
	libraries/AP_DroneCAN/AP_DroneCAN.cpp
	libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp
	libraries/AP_DroneCAN/AP_DroneCAN_serial.cpp
)

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
target_include_directories(AP_DroneCAN PUBLIC ${AP_DroneCAN_INCLUDES})

set_target_properties(AP_DroneCAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_DroneCAN mavlink) #clean
target_link_libraries(AP_DroneCAN canard) #clean
target_link_libraries(AP_DroneCAN littlefs) #clean

#------------------------------------------------

set(AP_EFI_SOURCES #3
	libraries/AP_EFI/AP_EFI.h
	libraries/AP_EFI/AP_EFI_Backend.h
	libraries/AP_EFI/AP_EFI_Currawong_ECU.h
	libraries/AP_EFI/AP_EFI_DroneCAN.h
	libraries/AP_EFI/AP_EFI_Loweheiser.h
	libraries/AP_EFI/AP_EFI_MAV.h
	libraries/AP_EFI/AP_EFI_NWPMU.h
	libraries/AP_EFI/AP_EFI_Scripting.h
	libraries/AP_EFI/AP_EFI_Serial_Hirth.h
	libraries/AP_EFI/AP_EFI_Serial_Lutan.h
	libraries/AP_EFI/AP_EFI_Serial_MS.h
	libraries/AP_EFI/AP_EFI_State.h
	libraries/AP_EFI/AP_EFI_ThrottleLinearisation.h
	libraries/AP_EFI/AP_EFI_config.h
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
	libraries/AP_EFI/AP_EFI_ThrottleLinearisation.cpp
)

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
target_include_directories(AP_EFI PUBLIC ${AP_EFI_INCLUDES})

set_target_properties(AP_EFI PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_EFI mavlink) #clean
target_link_libraries(AP_EFI canard) #clean
target_link_libraries(AP_EFI littlefs) #clean

#------------------------------------------------

set(AP_ESC_Telem_SOURCES #3
	libraries/AP_ESC_Telem/AP_ESC_Telem.h
	libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.h
	libraries/AP_ESC_Telem/AP_ESC_Telem_SITL.h
	libraries/AP_ESC_Telem/AP_ESC_Telem_config.h
	libraries/AP_ESC_Telem/LogStructure.h
	libraries/AP_ESC_Telem/AP_ESC_Telem.cpp
	libraries/AP_ESC_Telem/AP_ESC_Telem_Backend.cpp
	libraries/AP_ESC_Telem/AP_ESC_Telem_SITL.cpp
)

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
target_include_directories(AP_ESC_Telem PUBLIC ${AP_ESC_Telem_INCLUDES})

set_target_properties(AP_ESC_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ESC_Telem mavlink) #clean
target_link_libraries(AP_ESC_Telem canard) #clean
target_link_libraries(AP_ESC_Telem littlefs) #clean

#------------------------------------------------

set(AP_ExternalAHRS_SOURCES #3
	libraries/AP_ExternalAHRS/AP_ExternalAHRS.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain5.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain7.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG_structs.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_VectorNav.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_backend.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_config.h
	libraries/AP_ExternalAHRS/MicroStrain_common.h
	libraries/AP_ExternalAHRS/AP_ExternalAHRS.cpp
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_InertialLabs.cpp
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain5.cpp
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_MicroStrain7.cpp
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_SBG.cpp
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_VectorNav.cpp
	libraries/AP_ExternalAHRS/AP_ExternalAHRS_backend.cpp
	libraries/AP_ExternalAHRS/MicroStrain_common.cpp
)

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
target_include_directories(AP_ExternalAHRS PUBLIC ${AP_ExternalAHRS_INCLUDES})

set_target_properties(AP_ExternalAHRS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ExternalAHRS mavlink) #clean
target_link_libraries(AP_ExternalAHRS canard) #clean
target_link_libraries(AP_ExternalAHRS littlefs) #clean

#------------------------------------------------

set(AP_ExternalControl_SOURCES #3
	libraries/AP_ExternalControl/AP_ExternalControl.h
	libraries/AP_ExternalControl/AP_ExternalControl_config.h
	libraries/AP_ExternalControl/AP_ExternalControl.cpp
)

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
target_include_directories(AP_ExternalControl PUBLIC ${AP_ExternalControl_INCLUDES})

set_target_properties(AP_ExternalControl PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ExternalControl mavlink) #clean
target_link_libraries(AP_ExternalControl canard) #clean
target_link_libraries(AP_ExternalControl littlefs) #clean

#------------------------------------------------

set(AP_FETtecOneWire_SOURCES #3
	libraries/AP_FETtecOneWire/AP_FETtecOneWire.h
	libraries/AP_FETtecOneWire/AP_FETtecOneWire.cpp
)

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
target_include_directories(AP_FETtecOneWire PUBLIC ${AP_FETtecOneWire_INCLUDES})

set_target_properties(AP_FETtecOneWire PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_FETtecOneWire mavlink) #clean
target_link_libraries(AP_FETtecOneWire canard) #clean
target_link_libraries(AP_FETtecOneWire littlefs) #clean

#------------------------------------------------

set(AP_Filesystem_SOURCES #3
	libraries/AP_Filesystem/AP_Filesystem.h
	libraries/AP_Filesystem/AP_Filesystem_ESP32.h
	libraries/AP_Filesystem/AP_Filesystem_FATFS.h
	libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.h
	libraries/AP_Filesystem/AP_Filesystem_Mission.h
	libraries/AP_Filesystem/AP_Filesystem_Param.h
	libraries/AP_Filesystem/AP_Filesystem_ROMFS.h
	libraries/AP_Filesystem/AP_Filesystem_Sys.h
	libraries/AP_Filesystem/AP_Filesystem_backend.h
	libraries/AP_Filesystem/AP_Filesystem_config.h
	libraries/AP_Filesystem/AP_Filesystem_posix.h
	libraries/AP_Filesystem/posix_compat.h
	libraries/AP_Filesystem/AP_Filesystem.cpp
	libraries/AP_Filesystem/AP_Filesystem_ESP32.cpp
	libraries/AP_Filesystem/AP_Filesystem_FATFS.cpp
	libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.cpp
	libraries/AP_Filesystem/AP_Filesystem_Mission.cpp
	libraries/AP_Filesystem/AP_Filesystem_Param.cpp
	libraries/AP_Filesystem/AP_Filesystem_ROMFS.cpp
	libraries/AP_Filesystem/AP_Filesystem_Sys.cpp
	libraries/AP_Filesystem/AP_Filesystem_backend.cpp
	libraries/AP_Filesystem/AP_Filesystem_posix.cpp
	libraries/AP_Filesystem/posix_compat.cpp
)

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
target_include_directories(AP_Filesystem PUBLIC ${AP_Filesystem_INCLUDES})

set_target_properties(AP_Filesystem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Filesystem mavlink) #clean
target_link_libraries(AP_Filesystem canard) #clean
target_link_libraries(AP_Filesystem littlefs) #clean

#------------------------------------------------

set(AP_FlashStorage_SOURCES #3
	libraries/AP_FlashStorage/AP_FlashStorage.h
	libraries/AP_FlashStorage/AP_FlashStorage.cpp
)

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
target_include_directories(AP_FlashStorage PUBLIC ${AP_FlashStorage_INCLUDES})

set_target_properties(AP_FlashStorage PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_FlashStorage mavlink) #clean
target_link_libraries(AP_FlashStorage canard) #clean
target_link_libraries(AP_FlashStorage littlefs) #clean

#------------------------------------------------

set(AP_Frsky_Telem_SOURCES #3
	libraries/AP_Frsky_Telem/AP_Frsky_Backend.h
	libraries/AP_Frsky_Telem/AP_Frsky_D.h
	libraries/AP_Frsky_Telem/AP_Frsky_MAVlite.h
	libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.h
	libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_MAVliteToSPort.h
	libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_Message.h
	libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_SPortToMAVlite.h
	libraries/AP_Frsky_Telem/AP_Frsky_Parameters.h
	libraries/AP_Frsky_Telem/AP_Frsky_SPort.h
	libraries/AP_Frsky_Telem/AP_Frsky_SPortParser.h
	libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.h
	libraries/AP_Frsky_Telem/AP_Frsky_Telem.h
	libraries/AP_Frsky_Telem/AP_Frsky_config.h
	libraries/AP_Frsky_Telem/AP_Frsky_Backend.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_D.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_MAVliteMsgHandler.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_MAVliteToSPort.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_Message.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_MAVlite_SPortToMAVlite.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_Parameters.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_SPort.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp
	libraries/AP_Frsky_Telem/AP_Frsky_Telem.cpp
)

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
target_include_directories(AP_Frsky_Telem PUBLIC ${AP_Frsky_Telem_INCLUDES})

set_target_properties(AP_Frsky_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Frsky_Telem mavlink) #clean
target_link_libraries(AP_Frsky_Telem canard) #clean
target_link_libraries(AP_Frsky_Telem littlefs) #clean

#------------------------------------------------

set(AP_GPS_SOURCES #3
	libraries/AP_GPS/AP_GPS.h
	libraries/AP_GPS/AP_GPS_Blended.h
	libraries/AP_GPS/AP_GPS_DroneCAN.h
	libraries/AP_GPS/AP_GPS_ERB.h
	libraries/AP_GPS/AP_GPS_ExternalAHRS.h
	libraries/AP_GPS/AP_GPS_FixType.h
	libraries/AP_GPS/AP_GPS_GSOF.h
	libraries/AP_GPS/AP_GPS_MAV.h
	libraries/AP_GPS/AP_GPS_MSP.h
	libraries/AP_GPS/AP_GPS_NMEA.h
	libraries/AP_GPS/AP_GPS_NOVA.h
	libraries/AP_GPS/AP_GPS_SBF.h
	libraries/AP_GPS/AP_GPS_SBP.h
	libraries/AP_GPS/AP_GPS_SBP2.h
	libraries/AP_GPS/AP_GPS_SIRF.h
	libraries/AP_GPS/AP_GPS_SITL.h
	libraries/AP_GPS/AP_GPS_UBLOX.h
	libraries/AP_GPS/AP_GPS_config.h
	libraries/AP_GPS/GPS_Backend.h
	libraries/AP_GPS/GPS_detect_state.h
	libraries/AP_GPS/LogStructure.h
	libraries/AP_GPS/LogStructure_SBP.h
	libraries/AP_GPS/MovingBase.h
	libraries/AP_GPS/RTCM3_Parser.h
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
	libraries/AP_GPS/RTCM3_Parser.cpp
)

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
target_include_directories(AP_GPS PUBLIC ${AP_GPS_INCLUDES})

set_target_properties(AP_GPS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_GPS mavlink) #clean
target_link_libraries(AP_GPS canard) #clean
target_link_libraries(AP_GPS littlefs) #clean

#------------------------------------------------

set(AP_GSOF_SOURCES #3
	libraries/AP_GSOF/AP_GSOF.h
	libraries/AP_GSOF/AP_GSOF_config.h
	libraries/AP_GSOF/AP_GSOF.cpp
)

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
target_include_directories(AP_GSOF PUBLIC ${AP_GSOF_INCLUDES})

set_target_properties(AP_GSOF PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_GSOF mavlink) #clean
target_link_libraries(AP_GSOF canard) #clean
target_link_libraries(AP_GSOF littlefs) #clean

#------------------------------------------------

set(AP_Generator_SOURCES #3
	libraries/AP_Generator/AP_Generator.h
	libraries/AP_Generator/AP_Generator_Backend.h
	libraries/AP_Generator/AP_Generator_Cortex.h
	libraries/AP_Generator/AP_Generator_IE_2400.h
	libraries/AP_Generator/AP_Generator_IE_650_800.h
	libraries/AP_Generator/AP_Generator_IE_FuelCell.h
	libraries/AP_Generator/AP_Generator_Loweheiser.h
	libraries/AP_Generator/AP_Generator_RichenPower.h
	libraries/AP_Generator/AP_Generator_config.h
	libraries/AP_Generator/AP_Generator.cpp
	libraries/AP_Generator/AP_Generator_Backend.cpp
	libraries/AP_Generator/AP_Generator_Cortex.cpp
	libraries/AP_Generator/AP_Generator_IE_2400.cpp
	libraries/AP_Generator/AP_Generator_IE_650_800.cpp
	libraries/AP_Generator/AP_Generator_IE_FuelCell.cpp
	libraries/AP_Generator/AP_Generator_Loweheiser.cpp
	libraries/AP_Generator/AP_Generator_RichenPower.cpp
)

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
target_include_directories(AP_Generator PUBLIC ${AP_Generator_INCLUDES})

set_target_properties(AP_Generator PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Generator mavlink) #clean
target_link_libraries(AP_Generator canard) #clean
target_link_libraries(AP_Generator littlefs) #clean

#------------------------------------------------

set(AP_Gripper_SOURCES #3
	libraries/AP_Gripper/AP_Gripper.h
	libraries/AP_Gripper/AP_Gripper_Backend.h
	libraries/AP_Gripper/AP_Gripper_EPM.h
	libraries/AP_Gripper/AP_Gripper_Servo.h
	libraries/AP_Gripper/AP_Gripper_config.h
	libraries/AP_Gripper/AP_Gripper.cpp
	libraries/AP_Gripper/AP_Gripper_Backend.cpp
	libraries/AP_Gripper/AP_Gripper_EPM.cpp
	libraries/AP_Gripper/AP_Gripper_Servo.cpp
)

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
target_include_directories(AP_Gripper PUBLIC ${AP_Gripper_INCLUDES})

set_target_properties(AP_Gripper PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Gripper mavlink) #clean
target_link_libraries(AP_Gripper canard) #clean
target_link_libraries(AP_Gripper littlefs) #clean

#------------------------------------------------

set(AP_Hott_Telem_SOURCES #3
	libraries/AP_Hott_Telem/AP_Hott_Telem.h
	libraries/AP_Hott_Telem/AP_Hott_Telem.cpp
)

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
target_include_directories(AP_Hott_Telem PUBLIC ${AP_Hott_Telem_INCLUDES})

set_target_properties(AP_Hott_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Hott_Telem mavlink) #clean
target_link_libraries(AP_Hott_Telem canard) #clean
target_link_libraries(AP_Hott_Telem littlefs) #clean

#------------------------------------------------

set(AP_ICEngine_SOURCES #3
	libraries/AP_ICEngine/AP_ICEngine.h
	libraries/AP_ICEngine/AP_ICEngine_TCA9554.h
	libraries/AP_ICEngine/AP_ICEngine_config.h
	libraries/AP_ICEngine/AP_ICEngine.cpp
	libraries/AP_ICEngine/AP_ICEngine_TCA9554.cpp
)

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
target_include_directories(AP_ICEngine PUBLIC ${AP_ICEngine_INCLUDES})

set_target_properties(AP_ICEngine PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ICEngine mavlink) #clean
target_link_libraries(AP_ICEngine canard) #clean
target_link_libraries(AP_ICEngine littlefs) #clean

#------------------------------------------------

set(AP_IOMCU_SOURCES #3
	libraries/AP_IOMCU/AP_IOMCU.h
	libraries/AP_IOMCU/AP_IOMCU.cpp
	libraries/AP_IOMCU/fw_uploader.cpp
)

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
target_include_directories(AP_IOMCU PUBLIC ${AP_IOMCU_INCLUDES})

set_target_properties(AP_IOMCU PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_IOMCU mavlink) #clean
target_link_libraries(AP_IOMCU canard) #clean
target_link_libraries(AP_IOMCU littlefs) #clean

#------------------------------------------------

set(AP_IRLock_SOURCES #3
	libraries/AP_IRLock/AP_IRLock.h
	libraries/AP_IRLock/AP_IRLock_I2C.h
	libraries/AP_IRLock/AP_IRLock_SITL.h
	libraries/AP_IRLock/AP_IRLock_SITL_Gazebo.h
	libraries/AP_IRLock/AP_IRLock_config.h
	libraries/AP_IRLock/AP_IRLock.cpp
	libraries/AP_IRLock/AP_IRLock_I2C.cpp
	libraries/AP_IRLock/AP_IRLock_SITL.cpp
	libraries/AP_IRLock/AP_IRLock_SITL_Gazebo.cpp
)

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
target_include_directories(AP_IRLock PUBLIC ${AP_IRLock_INCLUDES})

set_target_properties(AP_IRLock PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_IRLock mavlink) #clean
target_link_libraries(AP_IRLock canard) #clean
target_link_libraries(AP_IRLock littlefs) #clean

#------------------------------------------------

set(AP_InertialNav_SOURCES #3
	libraries/AP_InertialNav/AP_InertialNav.h
	libraries/AP_InertialNav/AP_InertialNav.cpp
)

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
target_include_directories(AP_InertialNav PUBLIC ${AP_InertialNav_INCLUDES})

set_target_properties(AP_InertialNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_InertialNav mavlink) #clean
target_link_libraries(AP_InertialNav canard) #clean
target_link_libraries(AP_InertialNav littlefs) #clean

#------------------------------------------------

set(AP_InertialSensor_SOURCES #3
	libraries/AP_InertialSensor/AP_InertialSensor.h
	libraries/AP_InertialSensor/AP_InertialSensor_ADIS1647x.h
	libraries/AP_InertialSensor/AP_InertialSensor_BMI055.h
	libraries/AP_InertialSensor/AP_InertialSensor_BMI088.h
	libraries/AP_InertialSensor/AP_InertialSensor_BMI160.h
	libraries/AP_InertialSensor/AP_InertialSensor_BMI270.h
	libraries/AP_InertialSensor/AP_InertialSensor_Backend.h
	libraries/AP_InertialSensor/AP_InertialSensor_ExternalAHRS.h
	libraries/AP_InertialSensor/AP_InertialSensor_Invensense.h
	libraries/AP_InertialSensor/AP_InertialSensor_Invensense_registers.h
	libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.h
	libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2_registers.h
	libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.h
	libraries/AP_InertialSensor/AP_InertialSensor_L3G4200D.h
	libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS0.h
	libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS1.h
	libraries/AP_InertialSensor/AP_InertialSensor_NONE.h
	libraries/AP_InertialSensor/AP_InertialSensor_Params.h
	libraries/AP_InertialSensor/AP_InertialSensor_SCHA63T.h
	libraries/AP_InertialSensor/AP_InertialSensor_SITL.h
	libraries/AP_InertialSensor/AP_InertialSensor_config.h
	libraries/AP_InertialSensor/AP_InertialSensor_rate_config.h
	libraries/AP_InertialSensor/AP_InertialSensor_tempcal.h
	libraries/AP_InertialSensor/AuxiliaryBus.h
	libraries/AP_InertialSensor/FastRateBuffer.h
	libraries/AP_InertialSensor/LogStructure.h
	libraries/AP_InertialSensor/AP_InertialSensor.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_ADIS1647x.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_BMI055.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_BMI088.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_BMI160.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_BMI270.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_ExternalAHRS.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp
	libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp
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
	libraries/AP_InertialSensor/BatchSampler.cpp
	libraries/AP_InertialSensor/FastRateBuffer.cpp
)

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
target_include_directories(AP_InertialSensor PUBLIC ${AP_InertialSensor_INCLUDES})

set_target_properties(AP_InertialSensor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_InertialSensor mavlink) #clean
target_link_libraries(AP_InertialSensor canard) #clean
target_link_libraries(AP_InertialSensor littlefs) #clean

#------------------------------------------------

set(AP_InternalError_SOURCES #3
	libraries/AP_InternalError/AP_InternalError.h
	libraries/AP_InternalError/AP_InternalError_config.h
	libraries/AP_InternalError/AP_InternalError.cpp
)

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
target_include_directories(AP_InternalError PUBLIC ${AP_InternalError_INCLUDES})

set_target_properties(AP_InternalError PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_InternalError mavlink) #clean
target_link_libraries(AP_InternalError canard) #clean
target_link_libraries(AP_InternalError littlefs) #clean

#------------------------------------------------

set(AP_JSButton_SOURCES #3
	libraries/AP_JSButton/AP_JSButton.h
	libraries/AP_JSButton/AP_JSButton.cpp
)

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
target_include_directories(AP_JSButton PUBLIC ${AP_JSButton_INCLUDES})

set_target_properties(AP_JSButton PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_JSButton mavlink) #clean
target_link_libraries(AP_JSButton canard) #clean
target_link_libraries(AP_JSButton littlefs) #clean

#------------------------------------------------

set(AP_JSON_SOURCES #3
	libraries/AP_JSON/AP_JSON.h
	libraries/AP_JSON/AP_JSON.cpp
)

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
target_include_directories(AP_JSON PUBLIC ${AP_JSON_INCLUDES})

set_target_properties(AP_JSON PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_JSON mavlink) #clean
target_link_libraries(AP_JSON canard) #clean
target_link_libraries(AP_JSON littlefs) #clean

#------------------------------------------------

set(AP_KDECAN_SOURCES #3
	libraries/AP_KDECAN/AP_KDECAN.h
	libraries/AP_KDECAN/AP_KDECAN_config.h
	libraries/AP_KDECAN/AP_KDECAN.cpp
)

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
target_include_directories(AP_KDECAN PUBLIC ${AP_KDECAN_INCLUDES})

set_target_properties(AP_KDECAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_KDECAN mavlink) #clean
target_link_libraries(AP_KDECAN canard) #clean
target_link_libraries(AP_KDECAN littlefs) #clean

#------------------------------------------------

set(AP_L1_Control_SOURCES #3
	libraries/AP_L1_Control/AP_L1_Control.h
	libraries/AP_L1_Control/AP_L1_Control.cpp
)

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
target_include_directories(AP_L1_Control PUBLIC ${AP_L1_Control_INCLUDES})

set_target_properties(AP_L1_Control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_L1_Control mavlink) #clean
target_link_libraries(AP_L1_Control canard) #clean
target_link_libraries(AP_L1_Control littlefs) #clean

#------------------------------------------------

set(AP_LTM_Telem_SOURCES #3
	libraries/AP_LTM_Telem/AP_LTM_Telem.h
	libraries/AP_LTM_Telem/AP_LTM_Telem.cpp
)

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
target_include_directories(AP_LTM_Telem PUBLIC ${AP_LTM_Telem_INCLUDES})

set_target_properties(AP_LTM_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_LTM_Telem mavlink) #clean
target_link_libraries(AP_LTM_Telem canard) #clean
target_link_libraries(AP_LTM_Telem littlefs) #clean

#------------------------------------------------

set(AP_Landing_SOURCES #3
	libraries/AP_Landing/AP_Landing.h
	libraries/AP_Landing/AP_Landing_Deepstall.h
	libraries/AP_Landing/AP_Landing_config.h
	libraries/AP_Landing/LogStructure.h
	libraries/AP_Landing/AP_Landing.cpp
	libraries/AP_Landing/AP_Landing_Deepstall.cpp
	libraries/AP_Landing/AP_Landing_Slope.cpp
)

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
target_include_directories(AP_Landing PUBLIC ${AP_Landing_INCLUDES})

set_target_properties(AP_Landing PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Landing mavlink) #clean
target_link_libraries(AP_Landing canard) #clean
target_link_libraries(AP_Landing littlefs) #clean

#------------------------------------------------

set(AP_LeakDetector_SOURCES #3
	libraries/AP_LeakDetector/AP_LeakDetector.h
	libraries/AP_LeakDetector/AP_LeakDetector_Analog.h
	libraries/AP_LeakDetector/AP_LeakDetector_Backend.h
	libraries/AP_LeakDetector/AP_LeakDetector_Digital.h
	libraries/AP_LeakDetector/AP_LeakDetector.cpp
	libraries/AP_LeakDetector/AP_LeakDetector_Analog.cpp
	libraries/AP_LeakDetector/AP_LeakDetector_Backend.cpp
	libraries/AP_LeakDetector/AP_LeakDetector_Digital.cpp
)

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
target_include_directories(AP_LeakDetector PUBLIC ${AP_LeakDetector_INCLUDES})

set_target_properties(AP_LeakDetector PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_LeakDetector mavlink) #clean
target_link_libraries(AP_LeakDetector canard) #clean
target_link_libraries(AP_LeakDetector littlefs) #clean

#------------------------------------------------

set(AP_LightWareSerial_SOURCES #3
	libraries/AP_LightWareSerial/AP_LightWareSerial.h
	libraries/AP_LightWareSerial/AP_LightWareSerial_config.h
	libraries/AP_LightWareSerial/AP_LightWareSerial.cpp
)

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
target_include_directories(AP_LightWareSerial PUBLIC ${AP_LightWareSerial_INCLUDES})

set_target_properties(AP_LightWareSerial PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_LightWareSerial mavlink) #clean
target_link_libraries(AP_LightWareSerial canard) #clean
target_link_libraries(AP_LightWareSerial littlefs) #clean

#------------------------------------------------

set(AP_Logger_SOURCES #3
	libraries/AP_Logger/AP_Logger.h
	libraries/AP_Logger/AP_Logger_Backend.h
	libraries/AP_Logger/AP_Logger_Block.h
	libraries/AP_Logger/AP_Logger_File.h
	libraries/AP_Logger/AP_Logger_Flash_JEDEC.h
	libraries/AP_Logger/AP_Logger_MAVLink.h
	libraries/AP_Logger/AP_Logger_W25NXX.h
	libraries/AP_Logger/AP_Logger_config.h
	libraries/AP_Logger/LogStructure.h
	libraries/AP_Logger/LoggerMessageWriter.h
	libraries/AP_Logger/AP_Logger.cpp
	libraries/AP_Logger/AP_Logger_Backend.cpp
	libraries/AP_Logger/AP_Logger_Block.cpp
	libraries/AP_Logger/AP_Logger_File.cpp
	libraries/AP_Logger/AP_Logger_Flash_JEDEC.cpp
	libraries/AP_Logger/AP_Logger_MAVLink.cpp
	libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp
	libraries/AP_Logger/AP_Logger_W25NXX.cpp
	libraries/AP_Logger/LogFile.cpp
	libraries/AP_Logger/LoggerMessageWriter.cpp
)

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
target_include_directories(AP_Logger PUBLIC ${AP_Logger_INCLUDES})

set_target_properties(AP_Logger PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Logger mavlink) #clean
target_link_libraries(AP_Logger canard) #clean
target_link_libraries(AP_Logger littlefs) #clean

#------------------------------------------------

set(AP_MSP_SOURCES #3
	libraries/AP_MSP/AP_MSP.h
	libraries/AP_MSP/AP_MSP_Telem_Backend.h
	libraries/AP_MSP/AP_MSP_Telem_DJI.h
	libraries/AP_MSP/AP_MSP_Telem_DisplayPort.h
	libraries/AP_MSP/AP_MSP_Telem_Generic.h
	libraries/AP_MSP/AP_MSP_config.h
	libraries/AP_MSP/msp.h
	libraries/AP_MSP/msp_osd.h
	libraries/AP_MSP/msp_protocol.h
	libraries/AP_MSP/msp_sbuf.h
	libraries/AP_MSP/msp_sensors.h
	libraries/AP_MSP/msp_version.h
	libraries/AP_MSP/AP_MSP.cpp
	libraries/AP_MSP/AP_MSP_Telem_Backend.cpp
	libraries/AP_MSP/AP_MSP_Telem_DJI.cpp
	libraries/AP_MSP/AP_MSP_Telem_DisplayPort.cpp
	libraries/AP_MSP/AP_MSP_Telem_Generic.cpp
	libraries/AP_MSP/msp.cpp
	libraries/AP_MSP/msp_sbuf.cpp
)

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
target_include_directories(AP_MSP PUBLIC ${AP_MSP_INCLUDES})

set_target_properties(AP_MSP PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_MSP mavlink) #clean
target_link_libraries(AP_MSP canard) #clean
target_link_libraries(AP_MSP littlefs) #clean

#------------------------------------------------

set(AP_Math_SOURCES #3
	libraries/AP_Math/AP_GeodesicGrid.h
	libraries/AP_Math/AP_Math.h
	libraries/AP_Math/SCurve.h
	libraries/AP_Math/SplineCurve.h
	libraries/AP_Math/chirp.h
	libraries/AP_Math/control.h
	libraries/AP_Math/crc.h
	libraries/AP_Math/definitions.h
	libraries/AP_Math/div1000.h
	libraries/AP_Math/ftype.h
	libraries/AP_Math/location.h
	libraries/AP_Math/matrix3.h
	libraries/AP_Math/matrixN.h
	libraries/AP_Math/polyfit.h
	libraries/AP_Math/polygon.h
	libraries/AP_Math/quaternion.h
	libraries/AP_Math/rotations.h
	libraries/AP_Math/spline5.h
	libraries/AP_Math/vector2.h
	libraries/AP_Math/vector3.h
	libraries/AP_Math/vectorN.h
	libraries/AP_Math/AP_GeodesicGrid.cpp
	libraries/AP_Math/AP_Math.cpp
	libraries/AP_Math/SCurve.cpp
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
	libraries/AP_Math/vector3.cpp
)

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
target_include_directories(AP_Math PUBLIC ${AP_Math_INCLUDES})

set_target_properties(AP_Math PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Math mavlink) #clean
target_link_libraries(AP_Math canard) #clean
target_link_libraries(AP_Math littlefs) #clean

#------------------------------------------------

set(AP_Mission_SOURCES #3
	libraries/AP_Mission/AP_Mission.h
	libraries/AP_Mission/AP_Mission_ChangeDetector.h
	libraries/AP_Mission/AP_Mission_config.h
	libraries/AP_Mission/LogStructure.h
	libraries/AP_Mission/AP_Mission.cpp
	libraries/AP_Mission/AP_Mission_ChangeDetector.cpp
	libraries/AP_Mission/AP_Mission_Commands.cpp
)

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
target_include_directories(AP_Mission PUBLIC ${AP_Mission_INCLUDES})

set_target_properties(AP_Mission PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Mission mavlink) #clean
target_link_libraries(AP_Mission canard) #clean
target_link_libraries(AP_Mission littlefs) #clean

#------------------------------------------------

set(AP_Module_SOURCES #3
	libraries/AP_Module/AP_Module.h
	libraries/AP_Module/AP_Module_Structures.h
	libraries/AP_Module/AP_Module.cpp
)

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
target_include_directories(AP_Module PUBLIC ${AP_Module_INCLUDES})

set_target_properties(AP_Module PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Module mavlink) #clean
target_link_libraries(AP_Module canard) #clean
target_link_libraries(AP_Module littlefs) #clean

#------------------------------------------------

set(AP_Motors_SOURCES #3
	libraries/AP_Motors/AP_Motors.h
	libraries/AP_Motors/AP_Motors6DOF.h
	libraries/AP_Motors/AP_MotorsCoax.h
	libraries/AP_Motors/AP_MotorsHeli.h
	libraries/AP_Motors/AP_MotorsHeli_Dual.h
	libraries/AP_Motors/AP_MotorsHeli_Quad.h
	libraries/AP_Motors/AP_MotorsHeli_RSC.h
	libraries/AP_Motors/AP_MotorsHeli_Single.h
	libraries/AP_Motors/AP_MotorsHeli_Swash.h
	libraries/AP_Motors/AP_MotorsMatrix.h
	libraries/AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h
	libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h
	libraries/AP_Motors/AP_MotorsMulticopter.h
	libraries/AP_Motors/AP_MotorsSingle.h
	libraries/AP_Motors/AP_MotorsTailsitter.h
	libraries/AP_Motors/AP_MotorsTri.h
	libraries/AP_Motors/AP_Motors_Class.h
	libraries/AP_Motors/AP_Motors_Thrust_Linearization.h
	libraries/AP_Motors/AP_Motors_config.h
	libraries/AP_Motors/AP_Motors6DOF.cpp
	libraries/AP_Motors/AP_MotorsCoax.cpp
	libraries/AP_Motors/AP_MotorsHeli.cpp
	libraries/AP_Motors/AP_MotorsHeli_Dual.cpp
	libraries/AP_Motors/AP_MotorsHeli_Quad.cpp
	libraries/AP_Motors/AP_MotorsHeli_RSC.cpp
	libraries/AP_Motors/AP_MotorsHeli_Single.cpp
	libraries/AP_Motors/AP_MotorsHeli_Swash.cpp
	libraries/AP_Motors/AP_MotorsMatrix.cpp
	libraries/AP_Motors/AP_MotorsMatrix_6DoF_Scripting.cpp
	libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.cpp
	libraries/AP_Motors/AP_MotorsMulticopter.cpp
	libraries/AP_Motors/AP_MotorsSingle.cpp
	libraries/AP_Motors/AP_MotorsTailsitter.cpp
	libraries/AP_Motors/AP_MotorsTri.cpp
	libraries/AP_Motors/AP_Motors_Class.cpp
	libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp
)

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
target_include_directories(AP_Motors PUBLIC ${AP_Motors_INCLUDES})

set_target_properties(AP_Motors PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Motors mavlink) #clean
target_link_libraries(AP_Motors canard) #clean
target_link_libraries(AP_Motors littlefs) #clean

#------------------------------------------------

set(AP_Mount_SOURCES #3
	libraries/AP_Mount/AP_Mount.h
	libraries/AP_Mount/AP_Mount_Alexmos.h
	libraries/AP_Mount/AP_Mount_Backend.h
	libraries/AP_Mount/AP_Mount_Backend_Serial.h
	libraries/AP_Mount/AP_Mount_CADDX.h
	libraries/AP_Mount/AP_Mount_MAVLink.h
	libraries/AP_Mount/AP_Mount_Params.h
	libraries/AP_Mount/AP_Mount_SToRM32.h
	libraries/AP_Mount/AP_Mount_SToRM32_serial.h
	libraries/AP_Mount/AP_Mount_Scripting.h
	libraries/AP_Mount/AP_Mount_Servo.h
	libraries/AP_Mount/AP_Mount_Siyi.h
	libraries/AP_Mount/AP_Mount_SoloGimbal.h
	libraries/AP_Mount/AP_Mount_Topotek.h
	libraries/AP_Mount/AP_Mount_Viewpro.h
	libraries/AP_Mount/AP_Mount_XFRobot.h
	libraries/AP_Mount/AP_Mount_Xacti.h
	libraries/AP_Mount/AP_Mount_config.h
	libraries/AP_Mount/LogStructure.h
	libraries/AP_Mount/SoloGimbal.h
	libraries/AP_Mount/SoloGimbalEKF.h
	libraries/AP_Mount/SoloGimbal_Parameters.h
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
	libraries/AP_Mount/SoloGimbal_Parameters.cpp
)

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
target_include_directories(AP_Mount PUBLIC ${AP_Mount_INCLUDES})

set_target_properties(AP_Mount PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Mount mavlink) #clean
target_link_libraries(AP_Mount canard) #clean
target_link_libraries(AP_Mount littlefs) #clean

#------------------------------------------------

set(AP_MultiHeap_SOURCES #3
	libraries/AP_MultiHeap/AP_MultiHeap.h
	libraries/AP_MultiHeap/AP_MultiHeap.cpp
	libraries/AP_MultiHeap/MultiHeap_chibios.cpp
	libraries/AP_MultiHeap/MultiHeap_malloc.cpp
)

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
target_include_directories(AP_MultiHeap PUBLIC ${AP_MultiHeap_INCLUDES})

set_target_properties(AP_MultiHeap PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_MultiHeap mavlink) #clean
target_link_libraries(AP_MultiHeap canard) #clean
target_link_libraries(AP_MultiHeap littlefs) #clean

#------------------------------------------------

set(AP_NMEA_Output_SOURCES #3
	libraries/AP_NMEA_Output/AP_NMEA_Output.h
	libraries/AP_NMEA_Output/AP_NMEA_Output_config.h
	libraries/AP_NMEA_Output/AP_NMEA_Output.cpp
)

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
target_include_directories(AP_NMEA_Output PUBLIC ${AP_NMEA_Output_INCLUDES})

set_target_properties(AP_NMEA_Output PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_NMEA_Output mavlink) #clean
target_link_libraries(AP_NMEA_Output canard) #clean
target_link_libraries(AP_NMEA_Output littlefs) #clean

#------------------------------------------------

set(AP_NavEKF_SOURCES #3
	libraries/AP_NavEKF/AP_NavEKF_Source.h
	libraries/AP_NavEKF/AP_NavEKF_core_common.h
	libraries/AP_NavEKF/AP_Nav_Common.h
	libraries/AP_NavEKF/EKFGSF_yaw.h
	libraries/AP_NavEKF/EKF_Buffer.h
	libraries/AP_NavEKF/LogStructure.h
	libraries/AP_NavEKF/AP_NavEKF_Source.cpp
	libraries/AP_NavEKF/AP_NavEKF_core_common.cpp
	libraries/AP_NavEKF/EKFGSF_Logging.cpp
	libraries/AP_NavEKF/EKFGSF_yaw.cpp
	libraries/AP_NavEKF/EKF_Buffer.cpp
)

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
target_include_directories(AP_NavEKF PUBLIC ${AP_NavEKF_INCLUDES})

set_target_properties(AP_NavEKF PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_NavEKF mavlink) #clean
target_link_libraries(AP_NavEKF canard) #clean
target_link_libraries(AP_NavEKF littlefs) #clean

#------------------------------------------------

set(AP_NavEKF2_SOURCES #3
	libraries/AP_NavEKF2/AP_NavEKF2.h
	libraries/AP_NavEKF2/AP_NavEKF2_core.h
	libraries/AP_NavEKF2/LogStructure.h
	libraries/AP_NavEKF2/AP_NavEKF2.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_AirDataFusion.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_Control.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_Logging.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_MagFusion.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_Measurements.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_Outputs.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_PosVelFusion.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_RngBcnFusion.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_VehicleStatus.cpp
	libraries/AP_NavEKF2/AP_NavEKF2_core.cpp
	libraries/AP_NavEKF2/AP_NavEKF_GyroBias.cpp
)

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
target_include_directories(AP_NavEKF2 PUBLIC ${AP_NavEKF2_INCLUDES})

set_target_properties(AP_NavEKF2 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_NavEKF2 mavlink) #clean
target_link_libraries(AP_NavEKF2 canard) #clean
target_link_libraries(AP_NavEKF2 littlefs) #clean

#------------------------------------------------

set(AP_Networking_SOURCES #3
	libraries/AP_Networking/AP_Networking.h
	libraries/AP_Networking/AP_Networking_Backend.h
	libraries/AP_Networking/AP_Networking_CAN.h
	libraries/AP_Networking/AP_Networking_ChibiOS.h
	libraries/AP_Networking/AP_Networking_Config.h
	libraries/AP_Networking/AP_Networking_PPP.h
	libraries/AP_Networking/AP_Networking_SITL.h
	libraries/AP_Networking/AP_Networking_address.h
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
	libraries/AP_Networking/lwip_hal/arch/sys_arch.cpp
)

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
target_include_directories(AP_Networking PUBLIC ${AP_Networking_INCLUDES})

set_target_properties(AP_Networking PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Networking mavlink) #clean
target_link_libraries(AP_Networking canard) #clean
target_link_libraries(AP_Networking littlefs) #clean

#------------------------------------------------

set(AP_Notify_SOURCES #3
	libraries/AP_Notify/AP_BoardLED.h
	libraries/AP_Notify/AP_BoardLED2.h
	libraries/AP_Notify/AP_Notify.h
	libraries/AP_Notify/AP_Notify_config.h
	libraries/AP_Notify/Buzzer.h
	libraries/AP_Notify/DShotLED.h
	libraries/AP_Notify/DiscoLED.h
	libraries/AP_Notify/DiscreteRGBLed.h
	libraries/AP_Notify/Display.h
	libraries/AP_Notify/Display_Backend.h
	libraries/AP_Notify/Display_SH1106_I2C.h
	libraries/AP_Notify/Display_SITL.h
	libraries/AP_Notify/Display_SSD1306_I2C.h
	libraries/AP_Notify/DroneCAN_RGB_LED.h
	libraries/AP_Notify/ExternalLED.h
	libraries/AP_Notify/GPIO_LED_1.h
	libraries/AP_Notify/IS31FL3195.h
	libraries/AP_Notify/LP5562.h
	libraries/AP_Notify/Led_Sysfs.h
	libraries/AP_Notify/MMLPlayer.h
	libraries/AP_Notify/NCP5623.h
	libraries/AP_Notify/NavigatorLED.h
	libraries/AP_Notify/NeoPixel.h
	libraries/AP_Notify/NotifyDevice.h
	libraries/AP_Notify/OreoLED_I2C.h
	libraries/AP_Notify/PCA9685LED_I2C.h
	libraries/AP_Notify/PixRacerLED.h
	libraries/AP_Notify/ProfiLED.h
	libraries/AP_Notify/ProfiLED_IOMCU.h
	libraries/AP_Notify/RCOutputRGBLed.h
	libraries/AP_Notify/RGBLed.h
	libraries/AP_Notify/SITL_SFML_LED.h
	libraries/AP_Notify/ScriptingLED.h
	libraries/AP_Notify/SerialLED.h
	libraries/AP_Notify/ToneAlarm.h
	libraries/AP_Notify/ToshibaLED_I2C.h
	libraries/AP_Notify/VRBoard_LED.h
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
	libraries/AP_Notify/VRBoard_LED.cpp
)

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
target_include_directories(AP_Notify PUBLIC ${AP_Notify_INCLUDES})

set_target_properties(AP_Notify PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Notify mavlink) #clean
target_link_libraries(AP_Notify canard) #clean
target_link_libraries(AP_Notify littlefs) #clean

#------------------------------------------------

set(AP_OLC_SOURCES #3
	libraries/AP_OLC/AP_OLC.h
	libraries/AP_OLC/AP_OLC.cpp
)

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
target_include_directories(AP_OLC PUBLIC ${AP_OLC_INCLUDES})

set_target_properties(AP_OLC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_OLC mavlink) #clean
target_link_libraries(AP_OLC canard) #clean
target_link_libraries(AP_OLC littlefs) #clean

#------------------------------------------------

set(AP_OSD_SOURCES #3
	libraries/AP_OSD/AP_OSD.h
	libraries/AP_OSD/AP_OSD_Backend.h
	libraries/AP_OSD/AP_OSD_MAX7456.h
	libraries/AP_OSD/AP_OSD_MSP.h
	libraries/AP_OSD/AP_OSD_MSP_DisplayPort.h
	libraries/AP_OSD/AP_OSD_SITL.h
	libraries/AP_OSD/AP_OSD_config.h
	libraries/AP_OSD/AP_OSD.cpp
	libraries/AP_OSD/AP_OSD_Backend.cpp
	libraries/AP_OSD/AP_OSD_MAX7456.cpp
	libraries/AP_OSD/AP_OSD_MSP.cpp
	libraries/AP_OSD/AP_OSD_MSP_DisplayPort.cpp
	libraries/AP_OSD/AP_OSD_ParamScreen.cpp
	libraries/AP_OSD/AP_OSD_ParamSetting.cpp
	libraries/AP_OSD/AP_OSD_SITL.cpp
	libraries/AP_OSD/AP_OSD_Screen.cpp
	libraries/AP_OSD/AP_OSD_Setting.cpp
)

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
target_include_directories(AP_OSD PUBLIC ${AP_OSD_INCLUDES})

set_target_properties(AP_OSD PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_OSD mavlink) #clean
target_link_libraries(AP_OSD canard) #clean
target_link_libraries(AP_OSD littlefs) #clean

#------------------------------------------------

set(AP_OpenDroneID_SOURCES #3
	libraries/AP_OpenDroneID/AP_OpenDroneID.h
	libraries/AP_OpenDroneID/AP_OpenDroneID_config.h
	libraries/AP_OpenDroneID/AP_OpenDroneID.cpp
	libraries/AP_OpenDroneID/AP_OpenDroneID_DroneCAN.cpp
)

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
target_include_directories(AP_OpenDroneID PUBLIC ${AP_OpenDroneID_INCLUDES})

set_target_properties(AP_OpenDroneID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_OpenDroneID mavlink) #clean
target_link_libraries(AP_OpenDroneID canard) #clean
target_link_libraries(AP_OpenDroneID littlefs) #clean

#------------------------------------------------

set(AP_OpticalFlow_SOURCES #3
	libraries/AP_OpticalFlow/AP_OpticalFlow.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_Backend.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_CXOF.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_Calibrator.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_HereFlow.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_MSP.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_Onboard.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_PX4Flow.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_Pixart.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_Pixart_SROM.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_SITL.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_UPFLOW.h
	libraries/AP_OpticalFlow/AP_OpticalFlow_config.h
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
	libraries/AP_OpticalFlow/AP_OpticalFlow_UPFLOW.cpp
)

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
target_include_directories(AP_OpticalFlow PUBLIC ${AP_OpticalFlow_INCLUDES})

set_target_properties(AP_OpticalFlow PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_OpticalFlow mavlink) #clean
target_link_libraries(AP_OpticalFlow canard) #clean
target_link_libraries(AP_OpticalFlow littlefs) #clean

#------------------------------------------------

set(AP_Parachute_SOURCES #3
	libraries/AP_Parachute/AP_Parachute.h
	libraries/AP_Parachute/AP_Parachute_config.h
	libraries/AP_Parachute/AP_Parachute.cpp
)

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
target_include_directories(AP_Parachute PUBLIC ${AP_Parachute_INCLUDES})

set_target_properties(AP_Parachute PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Parachute mavlink) #clean
target_link_libraries(AP_Parachute canard) #clean
target_link_libraries(AP_Parachute littlefs) #clean

#------------------------------------------------

set(AP_Param_SOURCES #3
	libraries/AP_Param/AP_Param.h
	libraries/AP_Param/AP_Param_config.h
	libraries/AP_Param/AP_Param.cpp
	libraries/AP_Param/AP_ParamT.cpp
)

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
target_include_directories(AP_Param PUBLIC ${AP_Param_INCLUDES})

set_target_properties(AP_Param PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Param mavlink) #clean
target_link_libraries(AP_Param canard) #clean
target_link_libraries(AP_Param littlefs) #clean

#------------------------------------------------

set(AP_PiccoloCAN_SOURCES #3
	libraries/AP_PiccoloCAN/AP_PiccoloCAN.h
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_Cortex.h
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_Device.h
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_ECU.h
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_ESC.h
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_Servo.h
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_config.h
	libraries/AP_PiccoloCAN/AP_PiccoloCAN.cpp
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_Cortex.cpp
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_ECU.cpp
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_ESC.cpp
	libraries/AP_PiccoloCAN/AP_PiccoloCAN_Servo.cpp
)

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
target_include_directories(AP_PiccoloCAN PUBLIC ${AP_PiccoloCAN_INCLUDES})

set_target_properties(AP_PiccoloCAN PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_PiccoloCAN mavlink) #clean
target_link_libraries(AP_PiccoloCAN canard) #clean
target_link_libraries(AP_PiccoloCAN littlefs) #clean

#------------------------------------------------

set(AP_Proximity_SOURCES #3
	libraries/AP_Proximity/AP_Proximity.h
	libraries/AP_Proximity/AP_Proximity_AirSimSITL.h
	libraries/AP_Proximity/AP_Proximity_Backend.h
	libraries/AP_Proximity/AP_Proximity_Backend_Serial.h
	libraries/AP_Proximity/AP_Proximity_Boundary_3D.h
	libraries/AP_Proximity/AP_Proximity_Cygbot_D1.h
	libraries/AP_Proximity/AP_Proximity_DroneCAN.h
	libraries/AP_Proximity/AP_Proximity_LD06.h
	libraries/AP_Proximity/AP_Proximity_LightWareSF40C.h
	libraries/AP_Proximity/AP_Proximity_LightWareSF45B.h
	libraries/AP_Proximity/AP_Proximity_MAV.h
	libraries/AP_Proximity/AP_Proximity_MR72_CAN.h
	libraries/AP_Proximity/AP_Proximity_Params.h
	libraries/AP_Proximity/AP_Proximity_RPLidarA2.h
	libraries/AP_Proximity/AP_Proximity_RangeFinder.h
	libraries/AP_Proximity/AP_Proximity_SITL.h
	libraries/AP_Proximity/AP_Proximity_Scripting.h
	libraries/AP_Proximity/AP_Proximity_TeraRangerTower.h
	libraries/AP_Proximity/AP_Proximity_TeraRangerTowerEvo.h
	libraries/AP_Proximity/AP_Proximity_config.h
	libraries/AP_Proximity/LogStructure.h
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
	libraries/AP_Proximity/AP_Proximity_TeraRangerTowerEvo.cpp
	libraries/AP_Proximity/AP_Proximity_Utils.cpp
)

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
target_include_directories(AP_Proximity PUBLIC ${AP_Proximity_INCLUDES})

set_target_properties(AP_Proximity PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Proximity mavlink) #clean
target_link_libraries(AP_Proximity canard) #clean
target_link_libraries(AP_Proximity littlefs) #clean

#------------------------------------------------

set(AP_Quicktune_SOURCES #3
	libraries/AP_Quicktune/AP_Quicktune.h
	libraries/AP_Quicktune/AP_Quicktune.cpp
)

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
target_include_directories(AP_Quicktune PUBLIC ${AP_Quicktune_INCLUDES})

set_target_properties(AP_Quicktune PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Quicktune mavlink) #clean
target_link_libraries(AP_Quicktune canard) #clean
target_link_libraries(AP_Quicktune littlefs) #clean

#------------------------------------------------

set(AP_RAMTRON_SOURCES #3
	libraries/AP_RAMTRON/AP_RAMTRON.h
	libraries/AP_RAMTRON/AP_RAMTRON.cpp
)

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
target_include_directories(AP_RAMTRON PUBLIC ${AP_RAMTRON_INCLUDES})

set_target_properties(AP_RAMTRON PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RAMTRON mavlink) #clean
target_link_libraries(AP_RAMTRON canard) #clean
target_link_libraries(AP_RAMTRON littlefs) #clean

#------------------------------------------------

set(AP_RCMapper_SOURCES #3
	libraries/AP_RCMapper/AP_RCMapper.h
	libraries/AP_RCMapper/AP_RCMapper_config.h
	libraries/AP_RCMapper/AP_RCMapper.cpp
)

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
target_include_directories(AP_RCMapper PUBLIC ${AP_RCMapper_INCLUDES})

set_target_properties(AP_RCMapper PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RCMapper mavlink) #clean
target_link_libraries(AP_RCMapper canard) #clean
target_link_libraries(AP_RCMapper littlefs) #clean

#------------------------------------------------

set(AP_RCProtocol_SOURCES #3
	libraries/AP_RCProtocol/AP_RCProtocol.h
	libraries/AP_RCProtocol/AP_RCProtocol_Backend.h
	libraries/AP_RCProtocol/AP_RCProtocol_CRSF.h
	libraries/AP_RCProtocol/AP_RCProtocol_DSM.h
	libraries/AP_RCProtocol/AP_RCProtocol_DroneCAN.h
	libraries/AP_RCProtocol/AP_RCProtocol_Emlid_RCIO.h
	libraries/AP_RCProtocol/AP_RCProtocol_FDM.h
	libraries/AP_RCProtocol/AP_RCProtocol_FPort.h
	libraries/AP_RCProtocol/AP_RCProtocol_FPort2.h
	libraries/AP_RCProtocol/AP_RCProtocol_GHST.h
	libraries/AP_RCProtocol/AP_RCProtocol_IBUS.h
	libraries/AP_RCProtocol/AP_RCProtocol_IOMCU.h
	libraries/AP_RCProtocol/AP_RCProtocol_Joystick_SFML.h
	libraries/AP_RCProtocol/AP_RCProtocol_MAVLinkRadio.h
	libraries/AP_RCProtocol/AP_RCProtocol_PPMSum.h
	libraries/AP_RCProtocol/AP_RCProtocol_Radio.h
	libraries/AP_RCProtocol/AP_RCProtocol_SBUS.h
	libraries/AP_RCProtocol/AP_RCProtocol_SRXL.h
	libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.h
	libraries/AP_RCProtocol/AP_RCProtocol_ST24.h
	libraries/AP_RCProtocol/AP_RCProtocol_SUMD.h
	libraries/AP_RCProtocol/AP_RCProtocol_UDP.h
	libraries/AP_RCProtocol/AP_RCProtocol_config.h
	libraries/AP_RCProtocol/SoftSerial.h
	libraries/AP_RCProtocol/spm_srxl.h
	libraries/AP_RCProtocol/spm_srxl_config.h
	libraries/AP_RCProtocol/AP_RCProtocol.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_Backend.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_DSM.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_DroneCAN.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_Emlid_RCIO.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_FDM.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_FPort.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_FPort2.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_GHST.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_IBUS.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_IOMCU.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_Joystick_SFML.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_MAVLinkRadio.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_PPMSum.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_Radio.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_SBUS.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_SRXL.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_SRXL2.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_ST24.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_SUMD.cpp
	libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp
	libraries/AP_RCProtocol/SoftSerial.cpp
	libraries/AP_RCProtocol/spm_srxl.cpp
)

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
target_include_directories(AP_RCProtocol PUBLIC ${AP_RCProtocol_INCLUDES})

set_target_properties(AP_RCProtocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RCProtocol mavlink) #clean
target_link_libraries(AP_RCProtocol canard) #clean
target_link_libraries(AP_RCProtocol littlefs) #clean

#------------------------------------------------

set(AP_RCTelemetry_SOURCES #3
	libraries/AP_RCTelemetry/AP_CRSF_Telem.h
	libraries/AP_RCTelemetry/AP_GHST_Telem.h
	libraries/AP_RCTelemetry/AP_RCTelemetry.h
	libraries/AP_RCTelemetry/AP_RCTelemetry_config.h
	libraries/AP_RCTelemetry/AP_Spektrum_Telem.h
	libraries/AP_RCTelemetry/spektrumTelemetrySensors.h
	libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp
	libraries/AP_RCTelemetry/AP_GHST_Telem.cpp
	libraries/AP_RCTelemetry/AP_RCTelemetry.cpp
	libraries/AP_RCTelemetry/AP_Spektrum_Telem.cpp
)

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
target_include_directories(AP_RCTelemetry PUBLIC ${AP_RCTelemetry_INCLUDES})

set_target_properties(AP_RCTelemetry PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RCTelemetry mavlink) #clean
target_link_libraries(AP_RCTelemetry canard) #clean
target_link_libraries(AP_RCTelemetry littlefs) #clean

#------------------------------------------------

set(AP_ROMFS_SOURCES #3
	libraries/AP_ROMFS/AP_ROMFS.h
	libraries/AP_ROMFS/tinf.h
	libraries/AP_ROMFS/AP_ROMFS.cpp
	libraries/AP_ROMFS/tinflate.cpp
)

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
target_include_directories(AP_ROMFS PUBLIC ${AP_ROMFS_INCLUDES})

set_target_properties(AP_ROMFS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ROMFS mavlink) #clean
target_link_libraries(AP_ROMFS canard) #clean
target_link_libraries(AP_ROMFS littlefs) #clean

#------------------------------------------------

set(AP_RPM_SOURCES #3
	libraries/AP_RPM/AP_RPM.h
	libraries/AP_RPM/AP_RPM_Params.h
	libraries/AP_RPM/AP_RPM_config.h
	libraries/AP_RPM/LogStructure.h
	libraries/AP_RPM/RPM_Backend.h
	libraries/AP_RPM/RPM_DroneCAN.h
	libraries/AP_RPM/RPM_EFI.h
	libraries/AP_RPM/RPM_ESC_Telem.h
	libraries/AP_RPM/RPM_Generator.h
	libraries/AP_RPM/RPM_HarmonicNotch.h
	libraries/AP_RPM/RPM_Pin.h
	libraries/AP_RPM/RPM_SITL.h
	libraries/AP_RPM/AP_RPM.cpp
	libraries/AP_RPM/AP_RPM_Params.cpp
	libraries/AP_RPM/RPM_Backend.cpp
	libraries/AP_RPM/RPM_DroneCAN.cpp
	libraries/AP_RPM/RPM_EFI.cpp
	libraries/AP_RPM/RPM_ESC_Telem.cpp
	libraries/AP_RPM/RPM_Generator.cpp
	libraries/AP_RPM/RPM_HarmonicNotch.cpp
	libraries/AP_RPM/RPM_Pin.cpp
	libraries/AP_RPM/RPM_SITL.cpp
)

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
target_include_directories(AP_RPM PUBLIC ${AP_RPM_INCLUDES})

set_target_properties(AP_RPM PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RPM mavlink) #clean
target_link_libraries(AP_RPM canard) #clean
target_link_libraries(AP_RPM littlefs) #clean

#------------------------------------------------

set(AP_RSSI_SOURCES #3
	libraries/AP_RSSI/AP_RSSI.h
	libraries/AP_RSSI/AP_RSSI_config.h
	libraries/AP_RSSI/AP_RSSI.cpp
)

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
target_include_directories(AP_RSSI PUBLIC ${AP_RSSI_INCLUDES})

set_target_properties(AP_RSSI PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RSSI mavlink) #clean
target_link_libraries(AP_RSSI canard) #clean
target_link_libraries(AP_RSSI littlefs) #clean

#------------------------------------------------

set(AP_RTC_SOURCES #3
	libraries/AP_RTC/AP_RTC.h
	libraries/AP_RTC/AP_RTC_config.h
	libraries/AP_RTC/JitterCorrection.h
	libraries/AP_RTC/AP_RTC.cpp
	libraries/AP_RTC/JitterCorrection.cpp
)

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
target_include_directories(AP_RTC PUBLIC ${AP_RTC_INCLUDES})

set_target_properties(AP_RTC PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RTC mavlink) #clean
target_link_libraries(AP_RTC canard) #clean
target_link_libraries(AP_RTC littlefs) #clean

#------------------------------------------------

set(AP_Radio_SOURCES #3
	libraries/AP_Radio/AP_Radio.h
	libraries/AP_Radio/AP_Radio_backend.h
	libraries/AP_Radio/AP_Radio_bk2425.h
	libraries/AP_Radio/AP_Radio_cc2500.h
	libraries/AP_Radio/AP_Radio_config.h
	libraries/AP_Radio/AP_Radio_cypress.h
	libraries/AP_Radio/driver_bk2425.h
	libraries/AP_Radio/driver_cc2500.h
	libraries/AP_Radio/telem_structure.h
	libraries/AP_Radio/AP_Radio.cpp
	libraries/AP_Radio/AP_Radio_backend.cpp
	libraries/AP_Radio/AP_Radio_bk2425.cpp
	libraries/AP_Radio/AP_Radio_cc2500.cpp
	libraries/AP_Radio/AP_Radio_cypress.cpp
	libraries/AP_Radio/driver_bk2425.cpp
	libraries/AP_Radio/driver_cc2500.cpp
)

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
target_include_directories(AP_Radio PUBLIC ${AP_Radio_INCLUDES})

set_target_properties(AP_Radio PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Radio mavlink) #clean
target_link_libraries(AP_Radio canard) #clean
target_link_libraries(AP_Radio littlefs) #clean

#------------------------------------------------

set(AP_RangeFinder_SOURCES #3
	libraries/AP_RangeFinder/AP_RangeFinder.h
	libraries/AP_RangeFinder/AP_RangeFinder_Ainstein_LR_D1.h
	libraries/AP_RangeFinder/AP_RangeFinder_BBB_PRU.h
	libraries/AP_RangeFinder/AP_RangeFinder_BLPing.h
	libraries/AP_RangeFinder/AP_RangeFinder_Backend.h
	libraries/AP_RangeFinder/AP_RangeFinder_Backend_CAN.h
	libraries/AP_RangeFinder/AP_RangeFinder_Backend_Serial.h
	libraries/AP_RangeFinder/AP_RangeFinder_Bebop.h
	libraries/AP_RangeFinder/AP_RangeFinder_Benewake.h
	libraries/AP_RangeFinder/AP_RangeFinder_Benewake_CAN.h
	libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TF02.h
	libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TF03.h
	libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TFMini.h
	libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TFMiniPlus.h
	libraries/AP_RangeFinder/AP_RangeFinder_Benewake_TFS20L.h
	libraries/AP_RangeFinder/AP_RangeFinder_DroneCAN.h
	libraries/AP_RangeFinder/AP_RangeFinder_GYUS42v2.h
	libraries/AP_RangeFinder/AP_RangeFinder_HC_SR04.h
	libraries/AP_RangeFinder/AP_RangeFinder_JRE_Serial.h
	libraries/AP_RangeFinder/AP_RangeFinder_Lanbao.h
	libraries/AP_RangeFinder/AP_RangeFinder_LeddarOne.h
	libraries/AP_RangeFinder/AP_RangeFinder_LeddarVu8.h
	libraries/AP_RangeFinder/AP_RangeFinder_LightWareI2C.h
	libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h
	libraries/AP_RangeFinder/AP_RangeFinder_LightWare_GRF.h
	libraries/AP_RangeFinder/AP_RangeFinder_Lua.h
	libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.h
	libraries/AP_RangeFinder/AP_RangeFinder_MSP.h
	libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarI2CXL.h
	libraries/AP_RangeFinder/AP_RangeFinder_MaxsonarSerialLV.h
	libraries/AP_RangeFinder/AP_RangeFinder_NMEA.h
	libraries/AP_RangeFinder/AP_RangeFinder_NRA24_CAN.h
	libraries/AP_RangeFinder/AP_RangeFinder_NoopLoop.h
	libraries/AP_RangeFinder/AP_RangeFinder_PWM.h
	libraries/AP_RangeFinder/AP_RangeFinder_Params.h
	libraries/AP_RangeFinder/AP_RangeFinder_PulsedLightLRF.h
	libraries/AP_RangeFinder/AP_RangeFinder_RDS02UF.h
	libraries/AP_RangeFinder/AP_RangeFinder_SITL.h
	libraries/AP_RangeFinder/AP_RangeFinder_TOFSenseF_I2C.h
	libraries/AP_RangeFinder/AP_RangeFinder_TOFSenseP_CAN.h
	libraries/AP_RangeFinder/AP_RangeFinder_TeraRangerI2C.h
	libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.h
	libraries/AP_RangeFinder/AP_RangeFinder_USD1_CAN.h
	libraries/AP_RangeFinder/AP_RangeFinder_USD1_Serial.h
	libraries/AP_RangeFinder/AP_RangeFinder_VL53L0X.h
	libraries/AP_RangeFinder/AP_RangeFinder_VL53L1X.h
	libraries/AP_RangeFinder/AP_RangeFinder_Wasp.h
	libraries/AP_RangeFinder/AP_RangeFinder_analog.h
	libraries/AP_RangeFinder/AP_RangeFinder_config.h
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
	libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp
)

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
target_include_directories(AP_RangeFinder PUBLIC ${AP_RangeFinder_INCLUDES})

set_target_properties(AP_RangeFinder PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RangeFinder mavlink) #clean
target_link_libraries(AP_RangeFinder canard) #clean
target_link_libraries(AP_RangeFinder littlefs) #clean

#------------------------------------------------

set(AP_Relay_SOURCES #3
	libraries/AP_Relay/AP_Relay.h
	libraries/AP_Relay/AP_Relay_Params.h
	libraries/AP_Relay/AP_Relay_config.h
	libraries/AP_Relay/AP_Relay.cpp
	libraries/AP_Relay/AP_Relay_Params.cpp
)

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
target_include_directories(AP_Relay PUBLIC ${AP_Relay_INCLUDES})

set_target_properties(AP_Relay PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Relay mavlink) #clean
target_link_libraries(AP_Relay canard) #clean
target_link_libraries(AP_Relay littlefs) #clean

#------------------------------------------------

set(AP_RobotisServo_SOURCES #3
	libraries/AP_RobotisServo/AP_RobotisServo.h
	libraries/AP_RobotisServo/AP_RobotisServo.cpp
)

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
target_include_directories(AP_RobotisServo PUBLIC ${AP_RobotisServo_INCLUDES})

set_target_properties(AP_RobotisServo PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_RobotisServo mavlink) #clean
target_link_libraries(AP_RobotisServo canard) #clean
target_link_libraries(AP_RobotisServo littlefs) #clean

#------------------------------------------------

set(AP_SBusOut_SOURCES #3
	libraries/AP_SBusOut/AP_SBusOut.h
	libraries/AP_SBusOut/AP_SBusOut_config.h
	libraries/AP_SBusOut/AP_SBusOut.cpp
)

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
target_include_directories(AP_SBusOut PUBLIC ${AP_SBusOut_INCLUDES})

set_target_properties(AP_SBusOut PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_SBusOut mavlink) #clean
target_link_libraries(AP_SBusOut canard) #clean
target_link_libraries(AP_SBusOut littlefs) #clean

#------------------------------------------------

set(AP_Scheduler_SOURCES #3
	libraries/AP_Scheduler/AP_Scheduler.h
	libraries/AP_Scheduler/AP_Scheduler_config.h
	libraries/AP_Scheduler/PerfInfo.h
	libraries/AP_Scheduler/AP_Scheduler.cpp
	libraries/AP_Scheduler/PerfInfo.cpp
)

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
target_include_directories(AP_Scheduler PUBLIC ${AP_Scheduler_INCLUDES})

set_target_properties(AP_Scheduler PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Scheduler mavlink) #clean
target_link_libraries(AP_Scheduler canard) #clean
target_link_libraries(AP_Scheduler littlefs) #clean

#------------------------------------------------

set(AP_SerialLED_SOURCES #3
	libraries/AP_SerialLED/AP_SerialLED.h
	libraries/AP_SerialLED/AP_SerialLED_config.h
	libraries/AP_SerialLED/AP_SerialLED.cpp
)

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
target_include_directories(AP_SerialLED PUBLIC ${AP_SerialLED_INCLUDES})

set_target_properties(AP_SerialLED PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_SerialLED mavlink) #clean
target_link_libraries(AP_SerialLED canard) #clean
target_link_libraries(AP_SerialLED littlefs) #clean

#------------------------------------------------

set(AP_SerialManager_SOURCES #3
	libraries/AP_SerialManager/AP_SerialManager.h
	libraries/AP_SerialManager/AP_SerialManager_config.h
	libraries/AP_SerialManager/AP_SerialManager.cpp
)

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
target_include_directories(AP_SerialManager PUBLIC ${AP_SerialManager_INCLUDES})

set_target_properties(AP_SerialManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_SerialManager mavlink) #clean
target_link_libraries(AP_SerialManager canard) #clean
target_link_libraries(AP_SerialManager littlefs) #clean

#------------------------------------------------

set(AP_ServoRelayEvents_SOURCES #3
	libraries/AP_ServoRelayEvents/AP_ServoRelayEvents.h
	libraries/AP_ServoRelayEvents/AP_ServoRelayEvents_config.h
	libraries/AP_ServoRelayEvents/AP_ServoRelayEvents.cpp
)

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
target_include_directories(AP_ServoRelayEvents PUBLIC ${AP_ServoRelayEvents_INCLUDES})

set_target_properties(AP_ServoRelayEvents PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ServoRelayEvents mavlink) #clean
target_link_libraries(AP_ServoRelayEvents canard) #clean
target_link_libraries(AP_ServoRelayEvents littlefs) #clean

#------------------------------------------------

set(AP_Servo_Telem_SOURCES #3
	libraries/AP_Servo_Telem/AP_Servo_Telem.h
	libraries/AP_Servo_Telem/AP_Servo_Telem_config.h
	libraries/AP_Servo_Telem/LogStructure.h
	libraries/AP_Servo_Telem/AP_Servo_Telem.cpp
)

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
target_include_directories(AP_Servo_Telem PUBLIC ${AP_Servo_Telem_INCLUDES})

set_target_properties(AP_Servo_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Servo_Telem mavlink) #clean
target_link_libraries(AP_Servo_Telem canard) #clean
target_link_libraries(AP_Servo_Telem littlefs) #clean

#------------------------------------------------

set(AP_SmartRTL_SOURCES #3
	libraries/AP_SmartRTL/AP_SmartRTL.h
	libraries/AP_SmartRTL/AP_SmartRTL.cpp
)

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
target_include_directories(AP_SmartRTL PUBLIC ${AP_SmartRTL_INCLUDES})

set_target_properties(AP_SmartRTL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_SmartRTL mavlink) #clean
target_link_libraries(AP_SmartRTL canard) #clean
target_link_libraries(AP_SmartRTL littlefs) #clean

#------------------------------------------------

set(AP_Soaring_SOURCES #3
	libraries/AP_Soaring/AP_Soaring.h
	libraries/AP_Soaring/AP_Soaring_config.h
	libraries/AP_Soaring/ExtendedKalmanFilter.h
	libraries/AP_Soaring/SpeedToFly.h
	libraries/AP_Soaring/Variometer.h
	libraries/AP_Soaring/AP_Soaring.cpp
	libraries/AP_Soaring/ExtendedKalmanFilter.cpp
	libraries/AP_Soaring/SpeedToFly.cpp
	libraries/AP_Soaring/Variometer.cpp
)

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
target_include_directories(AP_Soaring PUBLIC ${AP_Soaring_INCLUDES})

set_target_properties(AP_Soaring PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Soaring mavlink) #clean
target_link_libraries(AP_Soaring canard) #clean
target_link_libraries(AP_Soaring littlefs) #clean

#------------------------------------------------

set(AP_Stats_SOURCES #3
	libraries/AP_Stats/AP_Stats.h
	libraries/AP_Stats/AP_Stats.cpp
)

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
target_include_directories(AP_Stats PUBLIC ${AP_Stats_INCLUDES})

set_target_properties(AP_Stats PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Stats mavlink) #clean
target_link_libraries(AP_Stats canard) #clean
target_link_libraries(AP_Stats littlefs) #clean

#------------------------------------------------

set(AP_SurfaceDistance_SOURCES #3
	libraries/AP_SurfaceDistance/AP_SurfaceDistance.h
	libraries/AP_SurfaceDistance/AP_SurfaceDistance.cpp
)

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
target_include_directories(AP_SurfaceDistance PUBLIC ${AP_SurfaceDistance_INCLUDES})

set_target_properties(AP_SurfaceDistance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_SurfaceDistance mavlink) #clean
target_link_libraries(AP_SurfaceDistance canard) #clean
target_link_libraries(AP_SurfaceDistance littlefs) #clean

#------------------------------------------------

set(AP_TECS_SOURCES #3
	libraries/AP_TECS/AP_TECS.h
	libraries/AP_TECS/AP_TECS.cpp
)

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
target_include_directories(AP_TECS PUBLIC ${AP_TECS_INCLUDES})

set_target_properties(AP_TECS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_TECS mavlink) #clean
target_link_libraries(AP_TECS canard) #clean
target_link_libraries(AP_TECS littlefs) #clean

#------------------------------------------------

set(AP_TempCalibration_SOURCES #3
	libraries/AP_TempCalibration/AP_TempCalibration.h
	libraries/AP_TempCalibration/AP_TempCalibration_config.h
	libraries/AP_TempCalibration/AP_TempCalibration.cpp
)

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
target_include_directories(AP_TempCalibration PUBLIC ${AP_TempCalibration_INCLUDES})

set_target_properties(AP_TempCalibration PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_TempCalibration mavlink) #clean
target_link_libraries(AP_TempCalibration canard) #clean
target_link_libraries(AP_TempCalibration littlefs) #clean

#------------------------------------------------

set(AP_TemperatureSensor_SOURCES #3
	libraries/AP_TemperatureSensor/AP_TemperatureSensor.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_Analog.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_Backend.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_DroneCAN.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_MAX31865.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_MCP9600.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_MLX90614.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_Params.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_SHT3x.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_TSYS01.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_TSYS03.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_config.h
	libraries/AP_TemperatureSensor/AP_TemperatureSensor.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_Analog.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_Backend.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_DroneCAN.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_MAX31865.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_MCP9600.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_MLX90614.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_Params.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_SHT3x.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_TSYS01.cpp
	libraries/AP_TemperatureSensor/AP_TemperatureSensor_TSYS03.cpp
)

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
target_include_directories(AP_TemperatureSensor PUBLIC ${AP_TemperatureSensor_INCLUDES})

set_target_properties(AP_TemperatureSensor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_TemperatureSensor mavlink) #clean
target_link_libraries(AP_TemperatureSensor canard) #clean
target_link_libraries(AP_TemperatureSensor littlefs) #clean

#------------------------------------------------

set(AP_Terrain_SOURCES #3
	libraries/AP_Terrain/AP_Terrain.h
	libraries/AP_Terrain/AP_Terrain_config.h
	libraries/AP_Terrain/AP_Terrain.cpp
	libraries/AP_Terrain/TerrainGCS.cpp
	libraries/AP_Terrain/TerrainIO.cpp
	libraries/AP_Terrain/TerrainMission.cpp
	libraries/AP_Terrain/TerrainUtil.cpp
)

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
target_include_directories(AP_Terrain PUBLIC ${AP_Terrain_INCLUDES})

set_target_properties(AP_Terrain PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Terrain mavlink) #clean
target_link_libraries(AP_Terrain canard) #clean
target_link_libraries(AP_Terrain littlefs) #clean

#------------------------------------------------

set(AP_Torqeedo_SOURCES #3
	libraries/AP_Torqeedo/AP_Torqeedo.h
	libraries/AP_Torqeedo/AP_Torqeedo_Backend.h
	libraries/AP_Torqeedo/AP_Torqeedo_Params.h
	libraries/AP_Torqeedo/AP_Torqeedo_TQBus.h
	libraries/AP_Torqeedo/AP_Torqeedo_config.h
	libraries/AP_Torqeedo/AP_Torqeedo.cpp
	libraries/AP_Torqeedo/AP_Torqeedo_Backend.cpp
	libraries/AP_Torqeedo/AP_Torqeedo_Params.cpp
	libraries/AP_Torqeedo/AP_Torqeedo_TQBus.cpp
)

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
target_include_directories(AP_Torqeedo PUBLIC ${AP_Torqeedo_INCLUDES})

set_target_properties(AP_Torqeedo PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Torqeedo mavlink) #clean
target_link_libraries(AP_Torqeedo canard) #clean
target_link_libraries(AP_Torqeedo littlefs) #clean

#------------------------------------------------

set(AP_Tuning_SOURCES #3
	libraries/AP_Tuning/AP_Tuning.h
	libraries/AP_Tuning/AP_Tuning_config.h
	libraries/AP_Tuning/AP_Tuning.cpp
)

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
target_include_directories(AP_Tuning PUBLIC ${AP_Tuning_INCLUDES})

set_target_properties(AP_Tuning PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Tuning mavlink) #clean
target_link_libraries(AP_Tuning canard) #clean
target_link_libraries(AP_Tuning littlefs) #clean

#------------------------------------------------

set(AP_Vehicle_SOURCES #3
	libraries/AP_Vehicle/AP_FixedWing.h
	libraries/AP_Vehicle/AP_MultiCopter.h
	libraries/AP_Vehicle/AP_Vehicle.h
	libraries/AP_Vehicle/AP_Vehicle_Type.h
	libraries/AP_Vehicle/AP_Vehicle_config.h
	libraries/AP_Vehicle/ModeReason.h
	libraries/AP_Vehicle/AP_Vehicle.cpp
	libraries/AP_Vehicle/Parameters.cpp
)

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
target_include_directories(AP_Vehicle PUBLIC ${AP_Vehicle_INCLUDES})

set_target_properties(AP_Vehicle PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Vehicle mavlink) #clean
target_link_libraries(AP_Vehicle canard) #clean
target_link_libraries(AP_Vehicle littlefs) #clean

#------------------------------------------------

set(AP_VideoTX_SOURCES #3
	libraries/AP_VideoTX/AP_SmartAudio.h
	libraries/AP_VideoTX/AP_Tramp.h
	libraries/AP_VideoTX/AP_VideoTX.h
	libraries/AP_VideoTX/AP_VideoTX_config.h
	libraries/AP_VideoTX/AP_SmartAudio.cpp
	libraries/AP_VideoTX/AP_Tramp.cpp
	libraries/AP_VideoTX/AP_VideoTX.cpp
)

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
target_include_directories(AP_VideoTX PUBLIC ${AP_VideoTX_INCLUDES})

set_target_properties(AP_VideoTX PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_VideoTX mavlink) #clean
target_link_libraries(AP_VideoTX canard) #clean
target_link_libraries(AP_VideoTX littlefs) #clean

#------------------------------------------------

set(AP_VisualOdom_SOURCES #3
	libraries/AP_VisualOdom/AP_VisualOdom.h
	libraries/AP_VisualOdom/AP_VisualOdom_Backend.h
	libraries/AP_VisualOdom/AP_VisualOdom_IntelT265.h
	libraries/AP_VisualOdom/AP_VisualOdom_MAV.h
	libraries/AP_VisualOdom/AP_VisualOdom_config.h
	libraries/AP_VisualOdom/LogStructure.h
	libraries/AP_VisualOdom/AP_VisualOdom.cpp
	libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp
	libraries/AP_VisualOdom/AP_VisualOdom_IntelT265.cpp
	libraries/AP_VisualOdom/AP_VisualOdom_Logging.cpp
	libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp
)

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
target_include_directories(AP_VisualOdom PUBLIC ${AP_VisualOdom_INCLUDES})

set_target_properties(AP_VisualOdom PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_VisualOdom mavlink) #clean
target_link_libraries(AP_VisualOdom canard) #clean
target_link_libraries(AP_VisualOdom littlefs) #clean

#------------------------------------------------

set(AP_Volz_Protocol_SOURCES #3
	libraries/AP_Volz_Protocol/AP_Volz_Protocol.h
	libraries/AP_Volz_Protocol/AP_Volz_Protocol.cpp
)

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
target_include_directories(AP_Volz_Protocol PUBLIC ${AP_Volz_Protocol_INCLUDES})

set_target_properties(AP_Volz_Protocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Volz_Protocol mavlink) #clean
target_link_libraries(AP_Volz_Protocol canard) #clean
target_link_libraries(AP_Volz_Protocol littlefs) #clean

#------------------------------------------------

set(AP_WheelEncoder_SOURCES #3
	libraries/AP_WheelEncoder/AP_WheelEncoder.h
	libraries/AP_WheelEncoder/AP_WheelRateControl.h
	libraries/AP_WheelEncoder/WheelEncoder_Backend.h
	libraries/AP_WheelEncoder/WheelEncoder_Quadrature.h
	libraries/AP_WheelEncoder/WheelEncoder_SITL_Quadrature.h
	libraries/AP_WheelEncoder/AP_WheelEncoder.cpp
	libraries/AP_WheelEncoder/AP_WheelRateControl.cpp
	libraries/AP_WheelEncoder/WheelEncoder_Backend.cpp
	libraries/AP_WheelEncoder/WheelEncoder_Quadrature.cpp
	libraries/AP_WheelEncoder/WheelEncoder_SITL_Quadrature.cpp
)

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
target_include_directories(AP_WheelEncoder PUBLIC ${AP_WheelEncoder_INCLUDES})

set_target_properties(AP_WheelEncoder PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_WheelEncoder mavlink) #clean
target_link_libraries(AP_WheelEncoder canard) #clean
target_link_libraries(AP_WheelEncoder littlefs) #clean

#------------------------------------------------

set(AP_Winch_SOURCES #3
	libraries/AP_Winch/AP_Winch.h
	libraries/AP_Winch/AP_Winch_Backend.h
	libraries/AP_Winch/AP_Winch_Daiwa.h
	libraries/AP_Winch/AP_Winch_PWM.h
	libraries/AP_Winch/AP_Winch_config.h
	libraries/AP_Winch/AP_Winch.cpp
	libraries/AP_Winch/AP_Winch_Backend.cpp
	libraries/AP_Winch/AP_Winch_Daiwa.cpp
	libraries/AP_Winch/AP_Winch_PWM.cpp
)

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
target_include_directories(AP_Winch PUBLIC ${AP_Winch_INCLUDES})

set_target_properties(AP_Winch PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Winch mavlink) #clean
target_link_libraries(AP_Winch canard) #clean
target_link_libraries(AP_Winch littlefs) #clean

#------------------------------------------------

set(AP_WindVane_SOURCES #3
	libraries/AP_WindVane/AP_WindVane.h
	libraries/AP_WindVane/AP_WindVane_Airspeed.h
	libraries/AP_WindVane/AP_WindVane_Analog.h
	libraries/AP_WindVane/AP_WindVane_Backend.h
	libraries/AP_WindVane/AP_WindVane_Home.h
	libraries/AP_WindVane/AP_WindVane_ModernDevice.h
	libraries/AP_WindVane/AP_WindVane_NMEA.h
	libraries/AP_WindVane/AP_WindVane_RPM.h
	libraries/AP_WindVane/AP_WindVane_SITL.h
	libraries/AP_WindVane/AP_WindVane_config.h
	libraries/AP_WindVane/AP_WindVane.cpp
	libraries/AP_WindVane/AP_WindVane_Airspeed.cpp
	libraries/AP_WindVane/AP_WindVane_Analog.cpp
	libraries/AP_WindVane/AP_WindVane_Backend.cpp
	libraries/AP_WindVane/AP_WindVane_Home.cpp
	libraries/AP_WindVane/AP_WindVane_ModernDevice.cpp
	libraries/AP_WindVane/AP_WindVane_NMEA.cpp
	libraries/AP_WindVane/AP_WindVane_RPM.cpp
	libraries/AP_WindVane/AP_WindVane_SITL.cpp
)

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
target_include_directories(AP_WindVane PUBLIC ${AP_WindVane_INCLUDES})

set_target_properties(AP_WindVane PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_WindVane mavlink) #clean
target_link_libraries(AP_WindVane canard) #clean
target_link_libraries(AP_WindVane littlefs) #clean

#------------------------------------------------

set(AR_Motors_SOURCES #3
	libraries/AR_Motors/AP_MotorsUGV.h
	libraries/AR_Motors/AP_MotorsUGV.cpp
)

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
target_include_directories(AR_Motors PUBLIC ${AR_Motors_INCLUDES})

set_target_properties(AR_Motors PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AR_Motors mavlink) #clean
target_link_libraries(AR_Motors canard) #clean
target_link_libraries(AR_Motors littlefs) #clean

#------------------------------------------------

set(AR_WPNav_SOURCES #3
	libraries/AR_WPNav/AR_PivotTurn.h
	libraries/AR_WPNav/AR_WPNav.h
	libraries/AR_WPNav/AR_WPNav_OA.h
	libraries/AR_WPNav/AR_PivotTurn.cpp
	libraries/AR_WPNav/AR_WPNav.cpp
	libraries/AR_WPNav/AR_WPNav_OA.cpp
)

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
target_include_directories(AR_WPNav PUBLIC ${AR_WPNav_INCLUDES})

set_target_properties(AR_WPNav PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AR_WPNav mavlink) #clean
target_link_libraries(AR_WPNav canard) #clean
target_link_libraries(AR_WPNav littlefs) #clean

#------------------------------------------------

set(Filter_SOURCES #3
	libraries/Filter/AP_Filter.h
	libraries/Filter/AP_Filter_config.h
	libraries/Filter/AverageFilter.h
	libraries/Filter/Butter.h
	libraries/Filter/DerivativeFilter.h
	libraries/Filter/Filter.h
	libraries/Filter/FilterClass.h
	libraries/Filter/FilterWithBuffer.h
	libraries/Filter/HarmonicNotchFilter.h
	libraries/Filter/LowPassFilter.h
	libraries/Filter/LowPassFilter2p.h
	libraries/Filter/ModeFilter.h
	libraries/Filter/NotchFilter.h
	libraries/Filter/SlewLimiter.h
	libraries/Filter/AP_Filter.cpp
	libraries/Filter/AP_Filter_params.cpp
	libraries/Filter/AP_NotchFilter_params.cpp
	libraries/Filter/DerivativeFilter.cpp
	libraries/Filter/HarmonicNotchFilter.cpp
	libraries/Filter/LowPassFilter.cpp
	libraries/Filter/LowPassFilter2p.cpp
	libraries/Filter/ModeFilter.cpp
	libraries/Filter/NotchFilter.cpp
	libraries/Filter/SlewLimiter.cpp
)

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
target_include_directories(Filter PUBLIC ${Filter_INCLUDES})

set_target_properties(Filter PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(Filter mavlink) #clean
target_link_libraries(Filter canard) #clean
target_link_libraries(Filter littlefs) #clean

#------------------------------------------------

set(GCS_MAVLink_SOURCES #3
	libraries/GCS_MAVLink/GCS.h
	libraries/GCS_MAVLink/GCS_Dummy.h
	libraries/GCS_MAVLink/GCS_FTP.h
	libraries/GCS_MAVLink/GCS_MAVLink.h
	libraries/GCS_MAVLink/GCS_config.h
	libraries/GCS_MAVLink/MAVLink_routing.h
	libraries/GCS_MAVLink/MissionItemProtocol.h
	libraries/GCS_MAVLink/MissionItemProtocol_Fence.h
	libraries/GCS_MAVLink/MissionItemProtocol_Rally.h
	libraries/GCS_MAVLink/MissionItemProtocol_Waypoints.h
	libraries/GCS_MAVLink/ap_message.h
	libraries/GCS_MAVLink/GCS.cpp
	libraries/GCS_MAVLink/GCS_Common.cpp
	libraries/GCS_MAVLink/GCS_DeviceOp.cpp
	libraries/GCS_MAVLink/GCS_FTP.cpp
	libraries/GCS_MAVLink/GCS_Fence.cpp
	libraries/GCS_MAVLink/GCS_MAVLink.cpp
	libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp
	libraries/GCS_MAVLink/GCS_Param.cpp
	libraries/GCS_MAVLink/GCS_Rally.cpp
	libraries/GCS_MAVLink/GCS_ServoRelay.cpp
	libraries/GCS_MAVLink/GCS_Signing.cpp
	libraries/GCS_MAVLink/GCS_serial_control.cpp
	libraries/GCS_MAVLink/MAVLink_routing.cpp
	libraries/GCS_MAVLink/MissionItemProtocol.cpp
	libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp
	libraries/GCS_MAVLink/MissionItemProtocol_Rally.cpp
	libraries/GCS_MAVLink/MissionItemProtocol_Waypoints.cpp
)

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
target_include_directories(GCS_MAVLink PUBLIC ${GCS_MAVLink_INCLUDES})

set_target_properties(GCS_MAVLink PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(GCS_MAVLink mavlink) #clean
target_link_libraries(GCS_MAVLink canard) #clean
target_link_libraries(GCS_MAVLink littlefs) #clean

#------------------------------------------------

set(PID_SOURCES #3
	libraries/PID/PID.h
	libraries/PID/PID.cpp
)

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
target_include_directories(PID PUBLIC ${PID_INCLUDES})

set_target_properties(PID PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(PID mavlink) #clean
target_link_libraries(PID canard) #clean
target_link_libraries(PID littlefs) #clean

#------------------------------------------------

set(RC_Channel_SOURCES #3
	libraries/RC_Channel/RC_Channel.h
	libraries/RC_Channel/RC_Channel_config.h
	libraries/RC_Channel/RC_Channels_VarInfo.h
	libraries/RC_Channel/RC_Channel.cpp
	libraries/RC_Channel/RC_Channels.cpp
)

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
target_include_directories(RC_Channel PUBLIC ${RC_Channel_INCLUDES})

set_target_properties(RC_Channel PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(RC_Channel mavlink) #clean
target_link_libraries(RC_Channel canard) #clean
target_link_libraries(RC_Channel littlefs) #clean

#------------------------------------------------

set(SRV_Channel_SOURCES #3
	libraries/SRV_Channel/SRV_Channel.h
	libraries/SRV_Channel/SRV_Channel_config.h
	libraries/SRV_Channel/SRV_Channel.cpp
	libraries/SRV_Channel/SRV_Channel_aux.cpp
	libraries/SRV_Channel/SRV_Channels.cpp
)

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
target_include_directories(SRV_Channel PUBLIC ${SRV_Channel_INCLUDES})

set_target_properties(SRV_Channel PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(SRV_Channel mavlink) #clean
target_link_libraries(SRV_Channel canard) #clean
target_link_libraries(SRV_Channel littlefs) #clean

#------------------------------------------------

set(AP_HAL_SOURCES #3
	libraries/AP_HAL/AP_HAL.h
	libraries/AP_HAL/AP_HAL_Boards.h
	libraries/AP_HAL/AP_HAL_Macros.h
	libraries/AP_HAL/AP_HAL_Main.h
	libraries/AP_HAL/AP_HAL_Namespace.h
	libraries/AP_HAL/AnalogIn.h
	libraries/AP_HAL/CANIface.h
	libraries/AP_HAL/DSP.h
	libraries/AP_HAL/Device.h
	libraries/AP_HAL/Flash.h
	libraries/AP_HAL/GPIO.h
	libraries/AP_HAL/HAL.h
	libraries/AP_HAL/I2CDevice.h
	libraries/AP_HAL/LogStructure.h
	libraries/AP_HAL/OpticalFlow.h
	libraries/AP_HAL/RCInput.h
	libraries/AP_HAL/RCOutput.h
	libraries/AP_HAL/SIMState.h
	libraries/AP_HAL/SPIDevice.h
	libraries/AP_HAL/Scheduler.h
	libraries/AP_HAL/Semaphores.h
	libraries/AP_HAL/Storage.h
	libraries/AP_HAL/UARTDriver.h
	libraries/AP_HAL/Util.h
	libraries/AP_HAL/WSPIDevice.h
	libraries/AP_HAL/system.h
	libraries/AP_HAL/CANIface.cpp
	libraries/AP_HAL/DSP.cpp
	libraries/AP_HAL/Device.cpp
	libraries/AP_HAL/GPIO.cpp
	libraries/AP_HAL/HAL.cpp
	libraries/AP_HAL/RCOutput.cpp
	libraries/AP_HAL/SIMState.cpp
	libraries/AP_HAL/Scheduler.cpp
	libraries/AP_HAL/Semaphores.cpp
	libraries/AP_HAL/Storage.cpp
	libraries/AP_HAL/UARTDriver.cpp
	libraries/AP_HAL/Util.cpp
	libraries/AP_HAL/system.cpp
	libraries/AP_HAL/utility/BetterStream.h
	libraries/AP_HAL/utility/DataRateLimit.h
	libraries/AP_HAL/utility/OwnPtr.h
	libraries/AP_HAL/utility/RCOutput_Tap.h
	libraries/AP_HAL/utility/RingBuffer.h
	libraries/AP_HAL/utility/Socket.h
	libraries/AP_HAL/utility/Socket_native.h
	libraries/AP_HAL/utility/dsm.h
	libraries/AP_HAL/utility/ftoa_engine.h
	libraries/AP_HAL/utility/functor.h
	libraries/AP_HAL/utility/getopt_cpp.h
	libraries/AP_HAL/utility/packetise.h
	libraries/AP_HAL/utility/print_vprintf.h
	libraries/AP_HAL/utility/replace.h
	libraries/AP_HAL/utility/sparse-endian.h
	libraries/AP_HAL/utility/srxl.h
	libraries/AP_HAL/utility/st24.h
	libraries/AP_HAL/utility/sumd.h
	libraries/AP_HAL/utility/xtoa_fast.h
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
	libraries/AP_HAL/utility/utoa_invert.cpp
)

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
target_include_directories(AP_HAL PUBLIC ${AP_HAL_INCLUDES})

set_target_properties(AP_HAL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_HAL mavlink) #clean
target_link_libraries(AP_HAL canard) #clean
target_link_libraries(AP_HAL littlefs) #clean

#------------------------------------------------

set(AP_HAL_Empty_SOURCES #3
	libraries/AP_HAL_Empty/AP_HAL_Empty.h
	libraries/AP_HAL_Empty/AP_HAL_Empty_Namespace.h
	libraries/AP_HAL_Empty/AP_HAL_Empty_Private.h
	libraries/AP_HAL_Empty/AnalogIn.h
	libraries/AP_HAL_Empty/DSP.h
	libraries/AP_HAL_Empty/Flash.h
	libraries/AP_HAL_Empty/GPIO.h
	libraries/AP_HAL_Empty/HAL_Empty_Class.h
	libraries/AP_HAL_Empty/I2CDevice.h
	libraries/AP_HAL_Empty/OpticalFlow.h
	libraries/AP_HAL_Empty/RCInput.h
	libraries/AP_HAL_Empty/RCOutput.h
	libraries/AP_HAL_Empty/SPIDevice.h
	libraries/AP_HAL_Empty/Scheduler.h
	libraries/AP_HAL_Empty/Semaphores.h
	libraries/AP_HAL_Empty/Storage.h
	libraries/AP_HAL_Empty/UARTDriver.h
	libraries/AP_HAL_Empty/Util.h
	libraries/AP_HAL_Empty/WSPIDevice.h
	libraries/AP_HAL_Empty/AnalogIn.cpp
	libraries/AP_HAL_Empty/GPIO.cpp
	libraries/AP_HAL_Empty/HAL_Empty_Class.cpp
	libraries/AP_HAL_Empty/RCInput.cpp
	libraries/AP_HAL_Empty/RCOutput.cpp
	libraries/AP_HAL_Empty/Semaphores.cpp
	libraries/AP_HAL_Empty/Storage.cpp
	libraries/AP_HAL_Empty/UARTDriver.cpp
)

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
target_include_directories(AP_HAL_Empty PUBLIC ${AP_HAL_Empty_INCLUDES})

set_target_properties(AP_HAL_Empty PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_HAL_Empty mavlink) #clean
target_link_libraries(AP_HAL_Empty canard) #clean
target_link_libraries(AP_HAL_Empty littlefs) #clean

#------------------------------------------------

set(AP_PiccoloCAN_piccolo_protocol_SOURCES #3
	libraries/AP_PiccoloCAN/piccolo_protocol/CortexDefines.h
	libraries/AP_PiccoloCAN/piccolo_protocol/CortexPackets.h
	libraries/AP_PiccoloCAN/piccolo_protocol/CortexProtocol.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ECUDefines.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ECUPackets.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ECUProtocol.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ECUSettings.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ESCCommands.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ESCDefines.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ESCPackets.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ESCVelocityProtocol.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ServoCommands.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ServoDefines.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ServoPackets.h
	libraries/AP_PiccoloCAN/piccolo_protocol/ServoProtocol.h
	libraries/AP_PiccoloCAN/piccolo_protocol/fielddecode.h
	libraries/AP_PiccoloCAN/piccolo_protocol/fieldencode.h
	libraries/AP_PiccoloCAN/piccolo_protocol/scaleddecode.h
	libraries/AP_PiccoloCAN/piccolo_protocol/scaledencode.h
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
	libraries/AP_PiccoloCAN/piccolo_protocol/scaledencode.c
)

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
target_include_directories(AP_PiccoloCAN_piccolo_protocol PUBLIC ${AP_PiccoloCAN_piccolo_protocol_INCLUDES})

set_target_properties(AP_PiccoloCAN_piccolo_protocol PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_PiccoloCAN_piccolo_protocol mavlink) #clean
target_link_libraries(AP_PiccoloCAN_piccolo_protocol canard) #clean
target_link_libraries(AP_PiccoloCAN_piccolo_protocol littlefs) #clean

#------------------------------------------------

set(AP_HAL_SITL_SOURCES #3
	libraries/AP_HAL_SITL/AP_HAL_SITL.h
	libraries/AP_HAL_SITL/AP_HAL_SITL_Namespace.h
	libraries/AP_HAL_SITL/AP_HAL_SITL_Private.h
	libraries/AP_HAL_SITL/AnalogIn.h
	libraries/AP_HAL_SITL/CANSocketIface.h
	libraries/AP_HAL_SITL/CAN_Multicast.h
	libraries/AP_HAL_SITL/CAN_SocketCAN.h
	libraries/AP_HAL_SITL/CAN_Transport.h
	libraries/AP_HAL_SITL/DSP.h
	libraries/AP_HAL_SITL/GPIO.h
	libraries/AP_HAL_SITL/HAL_SITL_Class.h
	libraries/AP_HAL_SITL/I2CDevice.h
	libraries/AP_HAL_SITL/RCInput.h
	libraries/AP_HAL_SITL/RCOutput.h
	libraries/AP_HAL_SITL/SITL_Periph_State.h
	libraries/AP_HAL_SITL/SITL_State.h
	libraries/AP_HAL_SITL/SITL_State_common.h
	libraries/AP_HAL_SITL/SPIDevice.h
	libraries/AP_HAL_SITL/Scheduler.h
	libraries/AP_HAL_SITL/Semaphores.h
	libraries/AP_HAL_SITL/Storage.h
	libraries/AP_HAL_SITL/ToneAlarm_SF.h
	libraries/AP_HAL_SITL/UARTDriver.h
	libraries/AP_HAL_SITL/Util.h
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
	libraries/AP_HAL_SITL/SITL_cmdline.cpp
	libraries/AP_HAL_SITL/SPIDevice.cpp
	libraries/AP_HAL_SITL/Scheduler.cpp
	libraries/AP_HAL_SITL/Semaphores.cpp
	libraries/AP_HAL_SITL/Storage.cpp
	libraries/AP_HAL_SITL/ToneAlarm_SF.cpp
	libraries/AP_HAL_SITL/UARTDriver.cpp
	libraries/AP_HAL_SITL/UART_utils.cpp
	libraries/AP_HAL_SITL/Util.cpp
	libraries/AP_HAL_SITL/sitl_airspeed.cpp
	libraries/AP_HAL_SITL/sitl_rangefinder.cpp
	libraries/AP_HAL_SITL/system.cpp
)

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
target_compile_definitions(AP_HAL_SITL PUBLIC -DAP_BUILD_TARGET_NAME="ArduCopter") #1
target_include_directories(AP_HAL_SITL PUBLIC ${AP_HAL_SITL_INCLUDES})

set_target_properties(AP_HAL_SITL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_HAL_SITL mavlink) #clean
target_link_libraries(AP_HAL_SITL canard) #clean
target_link_libraries(AP_HAL_SITL littlefs) #clean

#------------------------------------------------

set(SITL_SOURCES #3
	libraries/SITL/SIM_ADSB.h
	libraries/SITL/SIM_ADSB_Device.h
	libraries/SITL/SIM_ADSB_Sagetech_MXS.h
	libraries/SITL/SIM_AIS.h
	libraries/SITL/SIM_AS5600.h
	libraries/SITL/SIM_AirSim.h
	libraries/SITL/SIM_Aircraft.h
	libraries/SITL/SIM_Airspeed_DLVR.h
	libraries/SITL/SIM_BalanceBot.h
	libraries/SITL/SIM_Balloon.h
	libraries/SITL/SIM_BattMonitor_SMBus.h
	libraries/SITL/SIM_BattMonitor_SMBus_Generic.h
	libraries/SITL/SIM_BattMonitor_SMBus_Maxell.h
	libraries/SITL/SIM_BattMonitor_SMBus_Rotoye.h
	libraries/SITL/SIM_Battery.h
	libraries/SITL/SIM_Blimp.h
	libraries/SITL/SIM_Buzzer.h
	libraries/SITL/SIM_CRRCSim.h
	libraries/SITL/SIM_CRSF.h
	libraries/SITL/SIM_Calibration.h
	libraries/SITL/SIM_DroneCANDevice.h
	libraries/SITL/SIM_EFI_Hirth.h
	libraries/SITL/SIM_EFI_MegaSquirt.h
	libraries/SITL/SIM_ELRS.h
	libraries/SITL/SIM_FETtecOneWireESC.h
	libraries/SITL/SIM_FlightAxis.h
	libraries/SITL/SIM_Frame.h
	libraries/SITL/SIM_Frsky.h
	libraries/SITL/SIM_Frsky_D.h
	libraries/SITL/SIM_GPIO_LED_1.h
	libraries/SITL/SIM_GPIO_LED_2.h
	libraries/SITL/SIM_GPIO_LED_3.h
	libraries/SITL/SIM_GPIO_LED_RGB.h
	libraries/SITL/SIM_GPS.h
	libraries/SITL/SIM_GPS_FILE.h
	libraries/SITL/SIM_GPS_MSP.h
	libraries/SITL/SIM_GPS_NMEA.h
	libraries/SITL/SIM_GPS_NOVA.h
	libraries/SITL/SIM_GPS_SBF.h
	libraries/SITL/SIM_GPS_SBP.h
	libraries/SITL/SIM_GPS_SBP2.h
	libraries/SITL/SIM_GPS_SBP_Common.h
	libraries/SITL/SIM_GPS_Trimble.h
	libraries/SITL/SIM_GPS_UBLOX.h
	libraries/SITL/SIM_Gazebo.h
	libraries/SITL/SIM_GeneratorEngine.h
	libraries/SITL/SIM_Gimbal.h
	libraries/SITL/SIM_Glider.h
	libraries/SITL/SIM_Gripper_EPM.h
	libraries/SITL/SIM_Gripper_Servo.h
	libraries/SITL/SIM_Helicopter.h
	libraries/SITL/SIM_I2C.h
	libraries/SITL/SIM_I2CDevice.h
	libraries/SITL/SIM_ICEngine.h
	libraries/SITL/SIM_ICM40609.h
	libraries/SITL/SIM_INA3221.h
	libraries/SITL/SIM_IS31FL3195.h
	libraries/SITL/SIM_InertialLabs.h
	libraries/SITL/SIM_IntelligentEnergy.h
	libraries/SITL/SIM_IntelligentEnergy24.h
	libraries/SITL/SIM_Invensense_v3.h
	libraries/SITL/SIM_JEDEC.h
	libraries/SITL/SIM_JEDEC_MX25L3206E.h
	libraries/SITL/SIM_JSBSim.h
	libraries/SITL/SIM_JSON.h
	libraries/SITL/SIM_JSON_Master.h
	libraries/SITL/SIM_LED_n.h
	libraries/SITL/SIM_LM2755.h
	libraries/SITL/SIM_LP5562.h
	libraries/SITL/SIM_Loweheiser.h
	libraries/SITL/SIM_MS5525.h
	libraries/SITL/SIM_MS5611.h
	libraries/SITL/SIM_MS5XXX.h
	libraries/SITL/SIM_MaxSonarI2CXL.h
	libraries/SITL/SIM_MicroStrain.h
	libraries/SITL/SIM_Morse.h
	libraries/SITL/SIM_Motor.h
	libraries/SITL/SIM_MotorBoat.h
	libraries/SITL/SIM_Multicopter.h
	libraries/SITL/SIM_NoVehicle.h
	libraries/SITL/SIM_PS_LD06.h
	libraries/SITL/SIM_PS_LightWare.h
	libraries/SITL/SIM_PS_LightWare_SF45B.h
	libraries/SITL/SIM_PS_RPLidar.h
	libraries/SITL/SIM_PS_RPLidarA1.h
	libraries/SITL/SIM_PS_RPLidarA2.h
	libraries/SITL/SIM_PS_TeraRangerTower.h
	libraries/SITL/SIM_Parachute.h
	libraries/SITL/SIM_Plane.h
	libraries/SITL/SIM_Precland.h
	libraries/SITL/SIM_QMC5883L.h
	libraries/SITL/SIM_QuadPlane.h
	libraries/SITL/SIM_RAMTRON.h
	libraries/SITL/SIM_RAMTRON_FM25V02.h
	libraries/SITL/SIM_RF_Ainstein_LR_D1.h
	libraries/SITL/SIM_RF_Ainstein_LR_D1_v19.h
	libraries/SITL/SIM_RF_BLping.h
	libraries/SITL/SIM_RF_Benewake.h
	libraries/SITL/SIM_RF_Benewake_TF02.h
	libraries/SITL/SIM_RF_Benewake_TF03.h
	libraries/SITL/SIM_RF_Benewake_TFMiniPlus.h
	libraries/SITL/SIM_RF_Benewake_TFmini.h
	libraries/SITL/SIM_RF_GYUS42v2.h
	libraries/SITL/SIM_RF_JRE.h
	libraries/SITL/SIM_RF_Lanbao.h
	libraries/SITL/SIM_RF_LeddarOne.h
	libraries/SITL/SIM_RF_LightWareI2C_Legacy16Bit.h
	libraries/SITL/SIM_RF_LightWareSerial.h
	libraries/SITL/SIM_RF_LightWareSerialBinary.h
	libraries/SITL/SIM_RF_LightWare_GRF.h
	libraries/SITL/SIM_RF_MAVLink.h
	libraries/SITL/SIM_RF_MaxsonarSerialLV.h
	libraries/SITL/SIM_RF_NMEA.h
	libraries/SITL/SIM_RF_NoopLoop.h
	libraries/SITL/SIM_RF_RDS02UF.h
	libraries/SITL/SIM_RF_TeraRanger_Serial.h
	libraries/SITL/SIM_RF_USD1.h
	libraries/SITL/SIM_RF_USD1_v0.h
	libraries/SITL/SIM_RF_USD1_v1.h
	libraries/SITL/SIM_RF_Wasp.h
	libraries/SITL/SIM_RGBLED.h
	libraries/SITL/SIM_RichenPower.h
	libraries/SITL/SIM_Rover.h
	libraries/SITL/SIM_SMBusDevice.h
	libraries/SITL/SIM_SPI.h
	libraries/SITL/SIM_SPIDevice.h
	libraries/SITL/SIM_Sailboat.h
	libraries/SITL/SIM_Scrimmage.h
	libraries/SITL/SIM_SerialDevice.h
	libraries/SITL/SIM_SerialProximitySensor.h
	libraries/SITL/SIM_SerialRangeFinder.h
	libraries/SITL/SIM_Ship.h
	libraries/SITL/SIM_SilentWings.h
	libraries/SITL/SIM_SingleCopter.h
	libraries/SITL/SIM_SlungPayload.h
	libraries/SITL/SIM_SoloGimbal.h
	libraries/SITL/SIM_Sprayer.h
	libraries/SITL/SIM_StratoBlimp.h
	libraries/SITL/SIM_Submarine.h
	libraries/SITL/SIM_TFS20L.h
	libraries/SITL/SIM_Temperature_MCP9600.h
	libraries/SITL/SIM_Temperature_SHT3x.h
	libraries/SITL/SIM_Temperature_TSYS01.h
	libraries/SITL/SIM_Temperature_TSYS03.h
	libraries/SITL/SIM_TeraRangerI2C.h
	libraries/SITL/SIM_Tether.h
	libraries/SITL/SIM_ToneAlarm.h
	libraries/SITL/SIM_ToshibaLED.h
	libraries/SITL/SIM_Tracker.h
	libraries/SITL/SIM_VectorNav.h
	libraries/SITL/SIM_Vicon.h
	libraries/SITL/SIM_Volz.h
	libraries/SITL/SIM_Webots.h
	libraries/SITL/SIM_Webots_Python.h
	libraries/SITL/SIM_XPlane.h
	libraries/SITL/SIM_config.h
	libraries/SITL/SIM_last_letter.h
	libraries/SITL/SITL.h
	libraries/SITL/SITL_Input.h
	libraries/SITL/ServoModel.h
	libraries/SITL/SIM_ADSB.cpp
	libraries/SITL/SIM_ADSB_Sagetech_MXS.cpp
	libraries/SITL/SIM_AIS.cpp
	libraries/SITL/SIM_AS5600.cpp
	libraries/SITL/SIM_AirSim.cpp
	libraries/SITL/SIM_Aircraft.cpp
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
	libraries/SITL/SIM_DroneCANDevice.cpp
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
	libraries/SITL/SIM_XPlane.cpp
	libraries/SITL/SIM_last_letter.cpp
	libraries/SITL/SITL.cpp
	libraries/SITL/SITL_Airspeed.cpp
	libraries/SITL/SITL_Baro.cpp
	libraries/SITL/ServoModel.cpp
)

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
target_compile_definitions(SITL PUBLIC -DAP_BUILD_TARGET_NAME="ArduCopter") #1
target_include_directories(SITL PUBLIC ${SITL_INCLUDES})

set_target_properties(SITL PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(SITL mavlink) #clean
target_link_libraries(SITL canard) #clean
target_link_libraries(SITL littlefs) #clean

#------------------------------------------------

set(AP_Scripting_SOURCES #3
	libraries/AP_Scripting/AP_Scripting.h
	libraries/AP_Scripting/AP_Scripting_CANSensor.h
	libraries/AP_Scripting/AP_Scripting_CRSFMenu.h
	libraries/AP_Scripting/AP_Scripting_SerialAccess.h
	libraries/AP_Scripting/AP_Scripting_SerialDevice.h
	libraries/AP_Scripting/AP_Scripting_config.h
	libraries/AP_Scripting/AP_Scripting_helpers.h
	libraries/AP_Scripting/lua_bindings.h
	libraries/AP_Scripting/lua_boxed_numerics.h
	libraries/AP_Scripting/lua_common_defs.h
	libraries/AP_Scripting/lua_scripts.h
	libraries/AP_Scripting/AP_Scripting.cpp
	libraries/AP_Scripting/AP_Scripting_CANSensor.cpp
	libraries/AP_Scripting/AP_Scripting_CRSFMenu.cpp
	libraries/AP_Scripting/AP_Scripting_SerialAccess.cpp
	libraries/AP_Scripting/AP_Scripting_SerialDevice.cpp
	libraries/AP_Scripting/AP_Scripting_helpers.cpp
	libraries/AP_Scripting/lua_bindings.cpp
	libraries/AP_Scripting/lua_boxed_numerics.cpp
	libraries/AP_Scripting/lua_scripts.cpp
	build2/sitl/libraries/AP_Scripting/lua_generated_bindings.cpp #extra
)

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
target_include_directories(AP_Scripting PUBLIC ${AP_Scripting_INCLUDES})

set_target_properties(AP_Scripting PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Scripting mavlink) #clean
target_link_libraries(AP_Scripting canard) #clean
target_link_libraries(AP_Scripting littlefs) #clean

#------------------------------------------------

set(AP_Scripting_lua_src_SOURCES #3
	libraries/AP_Scripting/lua/src/lapi.h
	libraries/AP_Scripting/lua/src/lauxlib.h
	libraries/AP_Scripting/lua/src/lcode.h
	libraries/AP_Scripting/lua/src/lctype.h
	libraries/AP_Scripting/lua/src/ldebug.h
	libraries/AP_Scripting/lua/src/ldo.h
	libraries/AP_Scripting/lua/src/lfunc.h
	libraries/AP_Scripting/lua/src/lgc.h
	libraries/AP_Scripting/lua/src/llex.h
	libraries/AP_Scripting/lua/src/llimits.h
	libraries/AP_Scripting/lua/src/lmem.h
	libraries/AP_Scripting/lua/src/lobject.h
	libraries/AP_Scripting/lua/src/lopcodes.h
	libraries/AP_Scripting/lua/src/lparser.h
	libraries/AP_Scripting/lua/src/lprefix.h
	libraries/AP_Scripting/lua/src/lstate.h
	libraries/AP_Scripting/lua/src/lstring.h
	libraries/AP_Scripting/lua/src/ltable.h
	libraries/AP_Scripting/lua/src/ltm.h
	libraries/AP_Scripting/lua/src/lua.h
	libraries/AP_Scripting/lua/src/luaconf.h
	libraries/AP_Scripting/lua/src/lualib.h
	libraries/AP_Scripting/lua/src/lundump.h
	libraries/AP_Scripting/lua/src/lvm.h
	libraries/AP_Scripting/lua/src/lzio.h
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
	libraries/AP_Scripting/lua/src/lundump.c
	libraries/AP_Scripting/lua/src/lutf8lib.c
	libraries/AP_Scripting/lua/src/lvm.c
	libraries/AP_Scripting/lua/src/lzio.c
)

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
target_include_directories(AP_Scripting_lua_src PUBLIC ${AP_Scripting_lua_src_INCLUDES})

set_target_properties(AP_Scripting_lua_src PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Scripting_lua_src mavlink) #clean
target_link_libraries(AP_Scripting_lua_src canard) #clean
target_link_libraries(AP_Scripting_lua_src littlefs) #clean

#------------------------------------------------

set(AP_NavEKF3_SOURCES #3
	../libraries/AP_NavEKF3/AP_NavEKF3.h
	../libraries/AP_NavEKF3/AP_NavEKF3_core.h
	../libraries/AP_NavEKF3/AP_NavEKF3_feature.h
	../libraries/AP_NavEKF3/LogStructure.h
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
	../libraries/AP_NavEKF3/AP_NavEKF3_core.cpp
)

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
target_compile_definitions(AP_NavEKF3 PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_NavEKF3 PUBLIC ${AP_NavEKF3_INCLUDES})

set_target_properties(AP_NavEKF3 PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_NavEKF3 mavlink) #clean
target_link_libraries(AP_NavEKF3 canard) #clean
target_link_libraries(AP_NavEKF3 littlefs) #clean

#------------------------------------------------

set(AP_Rally_SOURCES #3
	../libraries/AP_Rally/AP_Rally.h
	../libraries/AP_Rally/AP_Rally_config.h
	../libraries/AP_Rally/AP_Rally.cpp
)

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
target_compile_definitions(AP_Rally PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_Rally PUBLIC ${AP_Rally_INCLUDES})

set_target_properties(AP_Rally PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Rally mavlink) #clean
target_link_libraries(AP_Rally canard) #clean
target_link_libraries(AP_Rally littlefs) #clean

#------------------------------------------------

set(StorageManager_SOURCES #3
	../libraries/StorageManager/StorageManager.h
	../libraries/StorageManager/StorageManager.cpp
)

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
target_compile_definitions(StorageManager PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(StorageManager PUBLIC ${StorageManager_INCLUDES})

set_target_properties(StorageManager PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(StorageManager mavlink) #clean
target_link_libraries(StorageManager canard) #clean
target_link_libraries(StorageManager littlefs) #clean

#------------------------------------------------

set(AP_IBus_Telem_SOURCES #3
	../libraries/AP_IBus_Telem/AP_IBus_Telem.h
	../libraries/AP_IBus_Telem/AP_IBus_Telem.cpp
)

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
target_compile_definitions(AP_IBus_Telem PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_IBus_Telem PUBLIC ${AP_IBus_Telem_INCLUDES})

set_target_properties(AP_IBus_Telem PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_IBus_Telem mavlink) #clean
target_link_libraries(AP_IBus_Telem canard) #clean
target_link_libraries(AP_IBus_Telem littlefs) #clean

#------------------------------------------------

set(AP_BLHeli_SOURCES #3
	../libraries/AP_BLHeli/AP_BLHeli.h
	../libraries/AP_BLHeli/blheli_4way_protocol.h
	../libraries/AP_BLHeli/AP_BLHeli.cpp
)

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
target_compile_definitions(AP_BLHeli PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_BLHeli PUBLIC ${AP_BLHeli_INCLUDES})

set_target_properties(AP_BLHeli PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_BLHeli mavlink) #clean
target_link_libraries(AP_BLHeli canard) #clean
target_link_libraries(AP_BLHeli littlefs) #clean

#------------------------------------------------

set(AC_Fence_SOURCES #3
	../libraries/AC_Fence/AC_Fence.h
	../libraries/AC_Fence/AC_Fence_config.h
	../libraries/AC_Fence/AC_PolyFence_loader.h
	../libraries/AC_Fence/LogStructure.h
	../libraries/AC_Fence/AC_Fence.cpp
	../libraries/AC_Fence/AC_PolyFence_loader.cpp
)

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
target_compile_definitions(AC_Fence PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AC_Fence PUBLIC ${AC_Fence_INCLUDES})

set_target_properties(AC_Fence PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AC_Fence mavlink) #clean
target_link_libraries(AC_Fence canard) #clean
target_link_libraries(AC_Fence littlefs) #clean

#------------------------------------------------

set(AP_LandingGear_SOURCES #3
	../libraries/AP_LandingGear/AP_LandingGear.h
	../libraries/AP_LandingGear/AP_LandingGear_config.h
	../libraries/AP_LandingGear/AP_LandingGear.cpp
)

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
target_compile_definitions(AP_LandingGear PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_LandingGear PUBLIC ${AP_LandingGear_INCLUDES})

set_target_properties(AP_LandingGear PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_LandingGear mavlink) #clean
target_link_libraries(AP_LandingGear canard) #clean
target_link_libraries(AP_LandingGear littlefs) #clean

#------------------------------------------------

set(AP_ADSB_sagetech-sdk_SOURCES #3
	../libraries/AP_ADSB/sagetech-sdk/sagetech_mxs.h
	../libraries/AP_ADSB/sagetech-sdk/sg.h
	../libraries/AP_ADSB/sagetech-sdk/sgUtil.h
	../libraries/AP_ADSB/sagetech-sdk/target.h
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
	../libraries/AP_ADSB/sagetech-sdk/uint322Buf.c
)

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
target_include_directories(AP_ADSB_sagetech-sdk PUBLIC ${AP_ADSB_sagetech-sdk_INCLUDES})

set_target_properties(AP_ADSB_sagetech-sdk PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_ADSB_sagetech-sdk mavlink) #clean
target_link_libraries(AP_ADSB_sagetech-sdk canard) #clean
target_link_libraries(AP_ADSB_sagetech-sdk littlefs) #clean

#------------------------------------------------

set(AP_GyroFFT_SOURCES #3
	../libraries/AP_GyroFFT/AP_GyroFFT.h
	../libraries/AP_GyroFFT/AP_GyroFFT.cpp
)

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
target_compile_definitions(AP_GyroFFT PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_GyroFFT PUBLIC ${AP_GyroFFT_INCLUDES})

set_target_properties(AP_GyroFFT PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_GyroFFT mavlink) #clean
target_link_libraries(AP_GyroFFT canard) #clean
target_link_libraries(AP_GyroFFT littlefs) #clean

#------------------------------------------------

set(AP_AIS_SOURCES #3
	../libraries/AP_AIS/AP_AIS.h
	../libraries/AP_AIS/AP_AIS_config.h
	../libraries/AP_AIS/LogStructure.h
	../libraries/AP_AIS/AP_AIS.cpp
)

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
target_compile_definitions(AP_AIS PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_AIS PUBLIC ${AP_AIS_INCLUDES})

set_target_properties(AP_AIS PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_AIS mavlink) #clean
target_link_libraries(AP_AIS canard) #clean
target_link_libraries(AP_AIS littlefs) #clean

#------------------------------------------------

set(AP_Arming_SOURCES #3
	../libraries/AP_Arming/AP_Arming.h
	../libraries/AP_Arming/AP_Arming_config.h
	../libraries/AP_Arming/AP_Arming.cpp
)

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
target_compile_definitions(AP_Arming PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_Arming PUBLIC ${AP_Arming_INCLUDES})

set_target_properties(AP_Arming PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Arming mavlink) #clean
target_link_libraries(AP_Arming canard) #clean
target_link_libraries(AP_Arming littlefs) #clean

#------------------------------------------------

set(AP_Follow_SOURCES #3
	../libraries/AP_Follow/AP_Follow.h
	../libraries/AP_Follow/AP_Follow_config.h
	../libraries/AP_Follow/AP_Follow.cpp
)

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
target_compile_definitions(AP_Follow PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #1
target_include_directories(AP_Follow PUBLIC ${AP_Follow_INCLUDES})

set_target_properties(AP_Follow PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Follow mavlink) #clean
target_link_libraries(AP_Follow canard) #clean
target_link_libraries(AP_Follow littlefs) #clean

#------------------------------------------------

set(AntennaTracker_libs_SOURCES #3
)

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
target_include_directories(AntennaTracker_libs PUBLIC ${AntennaTracker_libs_INCLUDES})

set_target_properties(AntennaTracker_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
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

set(tracker_SOURCES #3
	AntennaTracker/AP_Arming_Tracker.h
	AntennaTracker/GCS_MAVLink_Tracker.h
	AntennaTracker/GCS_Tracker.h
	AntennaTracker/Parameters.h
	AntennaTracker/RC_Channel_Tracker.h
	AntennaTracker/Tracker.h
	AntennaTracker/config.h
	AntennaTracker/defines.h
	AntennaTracker/mode.h
	AntennaTracker/version.h
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
	AntennaTracker/tracking.cpp
)

set(tracker_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(tracker ${tracker_SOURCES}) #2
target_compile_definitions(tracker PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_AntennaTracker -DAP_BUILD_TARGET_NAME="AntennaTracker") #2
target_link_directories(tracker PUBLIC build2) #2
#target_include_directories(tracker PUBLIC ${tracker_INCLUDES})


target_link_libraries(tracker AntennaTracker_libs) #clean

#------------------------------------------------

set(AP_Avoidance_SOURCES #3
	../libraries/AP_Avoidance/AP_Avoidance.h
	../libraries/AP_Avoidance/AP_Avoidance_config.h
	../libraries/AP_Avoidance/AP_Avoidance.cpp
)

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
target_compile_definitions(AP_Avoidance PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #1
target_include_directories(AP_Avoidance PUBLIC ${AP_Avoidance_INCLUDES})

set_target_properties(AP_Avoidance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
)


target_link_libraries(AP_Avoidance mavlink) #clean
target_link_libraries(AP_Avoidance canard) #clean
target_link_libraries(AP_Avoidance littlefs) #clean

#------------------------------------------------

set(ArduCopter_libs_SOURCES #3
)

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
target_include_directories(ArduCopter_libs PUBLIC ${ArduCopter_libs_INCLUDES})

set_target_properties(ArduCopter_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
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

set(copter_SOURCES #3
	ArduCopter/APM_Config.h
	ArduCopter/AP_Arming_Copter.h
	ArduCopter/AP_ExternalControl_Copter.h
	ArduCopter/AP_Rally.h
	ArduCopter/Copter.h
	ArduCopter/GCS_Copter.h
	ArduCopter/GCS_MAVLink_Copter.h
	ArduCopter/Parameters.h
	ArduCopter/RC_Channel_Copter.h
	ArduCopter/UserParameters.h
	ArduCopter/UserVariables.h
	ArduCopter/afs_copter.h
	ArduCopter/avoidance_adsb.h
	ArduCopter/config.h
	ArduCopter/defines.h
	ArduCopter/mode.h
	ArduCopter/toy_mode.h
	ArduCopter/version.h
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
	ArduCopter/tuning.cpp
)

set(copter_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(copter ${copter_SOURCES}) #2
target_compile_definitions(copter PUBLIC -DFRAME_CONFIG=MULTICOPTER_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_link_directories(copter PUBLIC build2) #2
#target_include_directories(copter PUBLIC ${copter_INCLUDES})


target_link_libraries(copter ArduCopter_libs) #clean
target_link_libraries(copter -Wl,--whole-archive mavlink -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive canard -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive dronecan_dsdlc_generated -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive littlefs -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_AttitudeControl -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_AutoTune -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_Autorotation -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_Avoidance -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_CustomControl -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_InputManager -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_PID -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_PrecLand -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_Sprayer -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_WPNav -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive APM_Control -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ADC -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ADSB -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_AHRS -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_AccelCal -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_AdvancedFailsafe -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Airspeed -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Baro -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_BattMonitor -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Beacon -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_BoardConfig -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Button -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_CANManager -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_CSVReader -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Camera -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_CheckFirmware -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Common -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Compass -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_CustomRotations -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_DAL -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_DDS -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Declination -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Devo_Telem -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_DroneCAN -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_EFI -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ESC_Telem -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ExternalAHRS -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ExternalControl -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_FETtecOneWire -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Filesystem -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_FlashStorage -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Frsky_Telem -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_GPS -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_GSOF -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Generator -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Gripper -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Hott_Telem -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ICEngine -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_IOMCU -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_IRLock -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_InertialNav -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_InertialSensor -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_InternalError -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_JSButton -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_JSON -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_KDECAN -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_L1_Control -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_LTM_Telem -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Landing -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_LeakDetector -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_LightWareSerial -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Logger -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_MSP -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Math -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Mission -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Module -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Motors -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Mount -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_MultiHeap -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_NMEA_Output -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_NavEKF -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_NavEKF2 -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Networking -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Notify -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_OLC -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_OSD -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_OpenDroneID -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_OpticalFlow -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Parachute -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Param -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_PiccoloCAN -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Proximity -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Quicktune -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RAMTRON -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RCMapper -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RCProtocol -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RCTelemetry -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ROMFS -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RPM -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RSSI -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RTC -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Radio -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RangeFinder -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Relay -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_RobotisServo -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_SBusOut -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Scheduler -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_SerialLED -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_SerialManager -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ServoRelayEvents -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Servo_Telem -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_SmartRTL -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Soaring -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Stats -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_SurfaceDistance -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_TECS -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_TempCalibration -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_TemperatureSensor -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Terrain -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Torqeedo -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Tuning -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Vehicle -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_VideoTX -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_VisualOdom -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Volz_Protocol -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_WheelEncoder -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Winch -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_WindVane -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AR_Motors -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AR_WPNav -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive Filter -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive GCS_MAVLink -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive PID -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive RC_Channel -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive SRV_Channel -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_HAL -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_HAL_Empty -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_PiccoloCAN_piccolo_protocol -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_HAL_SITL -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive SITL -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Scripting -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Scripting_lua_src -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_NavEKF3 -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Rally -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive StorageManager -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_IBus_Telem -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_BLHeli -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AC_Fence -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_LandingGear -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_ADSB_sagetech-sdk -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_GyroFFT -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_AIS -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Arming -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Follow -Wl,--no-whole-archive) #clean2
target_link_libraries(copter -Wl,--whole-archive AP_Avoidance -Wl,--no-whole-archive) #clean2
target_link_libraries(copter m)  # Math library
target_link_libraries(copter pthread)  # POSIX threads
set_target_properties(copter PROPERTIES 
    LINK_FLAGS "-Wl,--gc-sections -Wl,--wrap,malloc"
)

#------------------------------------------------

set(copter-heli_SOURCES #3
	ArduCopter/APM_Config.h
	ArduCopter/AP_Arming_Copter.h
	ArduCopter/AP_ExternalControl_Copter.h
	ArduCopter/AP_Rally.h
	ArduCopter/Copter.h
	ArduCopter/GCS_Copter.h
	ArduCopter/GCS_MAVLink_Copter.h
	ArduCopter/Parameters.h
	ArduCopter/RC_Channel_Copter.h
	ArduCopter/UserParameters.h
	ArduCopter/UserVariables.h
	ArduCopter/afs_copter.h
	ArduCopter/avoidance_adsb.h
	ArduCopter/config.h
	ArduCopter/defines.h
	ArduCopter/mode.h
	ArduCopter/toy_mode.h
	ArduCopter/version.h
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
	ArduCopter/tuning.cpp
)

set(copter-heli_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(copter-heli ${copter-heli_SOURCES}) #2
target_compile_definitions(copter-heli PUBLIC -DFRAME_CONFIG=HELI_FRAME -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_link_directories(copter-heli PUBLIC build2) #2
#target_include_directories(copter-heli PUBLIC ${copter-heli_INCLUDES})


target_link_libraries(copter-heli ArduCopter_libs) #clean

#------------------------------------------------

set(ArduPlane_libs_SOURCES #3
)

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
target_include_directories(ArduPlane_libs PUBLIC ${ArduPlane_libs_INCLUDES})

set_target_properties(ArduPlane_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
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

set(plane_SOURCES #3
	ArduPlane/AP_Arming_Plane.h
	ArduPlane/AP_ExternalControl_Plane.h
	ArduPlane/GCS_MAVLink_Plane.h
	ArduPlane/GCS_Plane.h
	ArduPlane/Parameters.h
	ArduPlane/Plane.h
	ArduPlane/RC_Channel_Plane.h
	ArduPlane/VTOL_Assist.h
	ArduPlane/afs_plane.h
	ArduPlane/avoidance_adsb.h
	ArduPlane/config.h
	ArduPlane/defines.h
	ArduPlane/mode.h
	ArduPlane/pullup.h
	ArduPlane/qautotune.h
	ArduPlane/quadplane.h
	ArduPlane/systemid.h
	ArduPlane/tailsitter.h
	ArduPlane/tiltrotor.h
	ArduPlane/transition.h
	ArduPlane/tuning.h
	ArduPlane/version.h
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
	ArduPlane/tuning.cpp
)

set(plane_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(plane ${plane_SOURCES}) #2
target_compile_definitions(plane PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane -DAP_BUILD_TARGET_NAME="ArduPlane") #2
target_link_directories(plane PUBLIC build2) #2
#target_include_directories(plane PUBLIC ${plane_INCLUDES})


target_link_libraries(plane ArduPlane_libs) #clean

#------------------------------------------------

set(ArduSub_libs_SOURCES #3
)

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
target_include_directories(ArduSub_libs PUBLIC ${ArduSub_libs_INCLUDES})

set_target_properties(ArduSub_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
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

set(sub_SOURCES #3
	ArduSub/AP_Arming_Sub.h
	ArduSub/GCS_MAVLink_Sub.h
	ArduSub/GCS_Sub.h
	ArduSub/Parameters.h
	ArduSub/RC_Channel_Sub.h
	ArduSub/Sub.h
	ArduSub/UserVariables.h
	ArduSub/actuators.h
	ArduSub/config.h
	ArduSub/defines.h
	ArduSub/mode.h
	ArduSub/script_button.h
	ArduSub/version.h
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
	ArduSub/turn_counter.cpp
)

set(sub_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(sub ${sub_SOURCES}) #2
target_compile_definitions(sub PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduSub -DAP_BUILD_TARGET_NAME="ArduSub") #2
target_link_directories(sub PUBLIC build2) #2
#target_include_directories(sub PUBLIC ${sub_INCLUDES})


target_link_libraries(sub ArduSub_libs) #clean

#------------------------------------------------

set(Blimp_libs_SOURCES #3
)

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
target_include_directories(Blimp_libs PUBLIC ${Blimp_libs_INCLUDES})

set_target_properties(Blimp_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
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

set(blimp_SOURCES #3
	Blimp/AP_Arming_Blimp.h
	Blimp/Blimp.h
	Blimp/Fins.h
	Blimp/GCS_Blimp.h
	Blimp/GCS_MAVLink_Blimp.h
	Blimp/Loiter.h
	Blimp/Parameters.h
	Blimp/RC_Channel_Blimp.h
	Blimp/config.h
	Blimp/defines.h
	Blimp/mode.h
	Blimp/version.h
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
	Blimp/system.cpp
)

set(blimp_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(blimp ${blimp_SOURCES}) #2
target_compile_definitions(blimp PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_Blimp -DAP_BUILD_TARGET_NAME="Blimp") #2
target_link_directories(blimp PUBLIC build2) #2
#target_include_directories(blimp PUBLIC ${blimp_INCLUDES})


target_link_libraries(blimp Blimp_libs) #clean

#------------------------------------------------

set(Rover_libs_SOURCES #3
)

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
target_include_directories(Rover_libs PUBLIC ${Rover_libs_INCLUDES})

set_target_properties(Rover_libs PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"
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

set(rover_SOURCES #3
	Rover/AP_Arming_Rover.h
	Rover/AP_ExternalControl_Rover.h
	Rover/AP_Rally.h
	Rover/GCS_MAVLink_Rover.h
	Rover/GCS_Rover.h
	Rover/Parameters.h
	Rover/RC_Channel_Rover.h
	Rover/Rover.h
	Rover/afs_rover.h
	Rover/config.h
	Rover/defines.h
	Rover/mode.h
	Rover/sailboat.h
	Rover/version.h
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
	Rover/system.cpp
)

set(rover_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_executable(rover ${rover_SOURCES}) #2
target_compile_definitions(rover PUBLIC -DAPM_BUILD_DIRECTORY=APM_BUILD_Rover -DAP_BUILD_TARGET_NAME="Rover") #2
target_link_directories(rover PUBLIC build2) #2
#target_include_directories(rover PUBLIC ${rover_INCLUDES})


target_link_libraries(rover Rover_libs) #clean

