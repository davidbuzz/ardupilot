
#------------------------------------------------

set(objs_AP_Avoidance_SOURCES
    ../libraries/AP_Avoidance/AP_Avoidance.cpp)

set(objs_AP_Avoidance_INCLUDES
    ../build/sitl/libraries
    ../build/sitl/libraries/GCS_MAVLink
    ../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    ../modules/DroneCAN/libcanard
    ../libraries/AP_DroneCAN/canard
    /home/buzz2/ardupilot/build/sitl
    /home/buzz2/ardupilot/libraries/
    /home/buzz2/ardupilot/modules/littlefs/
    /home/buzz2/ardupilot/libraries/AP_Common/missing
    /home/buzz2/ardupilot/modules/lwip/src/include
    /home/buzz2/ardupilot/libraries/AP_Networking/config
    /home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include
    /home/buzz2/ardupilot/build/sitl/libraries
    /home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink
    /home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include
    /home/buzz2/ardupilot/modules/DroneCAN/libcanard
    /home/buzz2/ardupilot/libraries/AP_DroneCAN/canard)

add_library(objs_AP_Avoidance ${objs_AP_Avoidance_SOURCES}) #2
target_compile_definitions(objs_AP_Avoidance PRIVATE -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BUILD_TARGET_NAME="ArduCopter") #2
target_include_directories(objs_AP_Avoidance PRIVATE ${objs_AP_Avoidance_INCLUDES})

set_target_properties(objs_AP_Avoidance PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)


target_link_libraries(objs_AP_Avoidance mavlink) #clean
target_link_libraries(objs_AP_Avoidance dronecan) #clean
target_link_libraries(objs_AP_Avoidance littlefs) #clean

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


target_link_libraries(ArduCopter_libs ../libobjs_AP_AccelCal.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_ADC.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_AHRS.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Airspeed.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Baro.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_BattMonitor.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_BoardConfig.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Camera.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Common.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Compass.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Declination.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_GPS.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_GSOF.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_HAL.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_HAL_Empty.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_DDS.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_InertialSensor.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Math.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Mission.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_DAL.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_NavEKF.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_NavEKF2.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_NavEKF3.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Notify.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_OpticalFlow.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Param.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Rally.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_LightWareSerial.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RangeFinder.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Scheduler.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_SerialManager.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Terrain.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Vehicle.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_InternalError.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Logger.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_Filter.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_GCS_MAVLink.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_RC_Channel.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_SRV_Channel.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_StorageManager.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Tuning.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RPM.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RSSI.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Mount.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Module.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Button.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_ICEngine.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Networking.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Frsky_Telem.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_IBus_Telem.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_FlashStorage.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Relay.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_ServoRelayEvents.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Volz_Protocol.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_SBusOut.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_IOMCU.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Parachute.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RAMTRON.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RCProtocol.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Radio.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_TempCalibration.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_VisualOdom.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_BLHeli.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_ROMFS.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Proximity.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Gripper.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RTC.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_Sprayer.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_Fence.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_Avoidance.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_LandingGear.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RobotisServo.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_NMEA_Output.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_OSD.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Filesystem.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_ADSB.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_PID.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_SerialLED.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_EFI.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Hott_Telem.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_ESC_Telem.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Servo_Telem.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Stats.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_GyroFFT.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RCTelemetry.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Generator.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_MSP.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_OLC.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_WheelEncoder.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_ExternalAHRS.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_VideoTX.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_FETtecOneWire.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_TemperatureSensor.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Torqeedo.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_CustomRotations.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_AIS.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_OpenDroneID.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_CheckFirmware.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_ExternalControl.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_JSON.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Beacon.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Arming.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_RCMapper.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_MultiHeap.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Follow.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_CANManager.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_KDECAN.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_PiccoloCAN.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_AttitudeControl.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_InputManager.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_PrecLand.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_Autorotation.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_WPNav.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_IRLock.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Motors.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Avoidance.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_AdvancedFailsafe.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_SmartRTL.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Winch.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_LTM_Telem.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Devo_Telem.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_AutoTune.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_SurfaceDistance.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_DroneCAN.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_HAL_SITL.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_CSVReader.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_SITL.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AP_Scripting.a) #stripped2
target_link_libraries(ArduCopter_libs ../libobjs_AC_CustomControl.a) #stripped2

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

