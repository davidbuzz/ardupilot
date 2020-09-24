#include "Plane.h"

#if LOGGING_ENABLED == ENABLED

// Write an attitude packet
void Plane::Log_Write_Attitude(void)
{
    Vector3f targets;       // Package up the targets into a vector for commonality with Copter usage of Log_Wrote_Attitude
    targets.x = nav_roll_cd;
    targets.y = nav_pitch_cd;

    if (quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) {
        // when VTOL active log the copter target yaw
        targets.z = wrap_360_cd(quadplane.attitude_control->get_att_target_euler_cd().z);
    } else {
        //Plane does not have the concept of navyaw. This is a placeholder.
        targets.z = 0;
    }

    if (quadplane.tailsitter_active() || quadplane.in_vtol_mode()) {
        // we need the attitude targets from the AC_AttitudeControl controller, as they
        // account for the acceleration limits.
        // Also, for bodyframe roll input types, _attitude_target_euler_angle is not maintained
        // since Euler angles are not used and it is a waste of cpu to compute them at the loop rate.
        // Get them from the quaternion instead:
        quadplane.attitude_control->get_attitude_target_quat().to_euler(targets.x, targets.y, targets.z);
        targets *= degrees(100.0f);
        logger.Write_AttitudeView(*quadplane.ahrs_view, targets);
    } else {
        logger.Write_Attitude(targets);
    }
    if (quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) {
        // log quadplane PIDs separately from fixed wing PIDs
        logger.Write_PID(LOG_PIQR_MSG, quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIQP_MSG, quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIQY_MSG, quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIQA_MSG, quadplane.pos_control->get_accel_z_pid().get_pid_info() );
    }

    logger.Write_PID(LOG_PIDR_MSG, rollController.get_pid_info());
    logger.Write_PID(LOG_PIDP_MSG, pitchController.get_pid_info());
    logger.Write_PID(LOG_PIDY_MSG, yawController.get_pid_info());
    logger.Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());

#if AP_AHRS_NAVEKF_AVAILABLE
    AP::ahrs_navekf().Log_Write();
    logger.Write_AHRS2();
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE();
#endif
    logger.Write_POS();
}

// do logging at loop rate
void Plane::Log_Write_Fast(void)
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
}


struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t startup_type;
    uint16_t command_total;
};

void Plane::Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        time_us         : AP_HAL::micros64(),
        startup_type    : type,
        command_total   : mission.num_commands()
    };
    logger.WriteCriticalBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    int16_t throttle_dem;
    float airspeed_estimate;
};

struct PACKED log_Guided {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float target_airspeed_cm;
    float target_airspeed_accel;
    float target_alt;
    float target_alt_accel;
    uint8_t target_alt_frame;
    float target_heading;
    float target_heading_limit;
};

// Write a control tuning packet. Total length : 22 bytes
void Plane::Log_Write_Control_Tuning()
{
    float est_airspeed = 0;
    ahrs.airspeed_estimate(&est_airspeed);
    
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_us         : AP_HAL::micros64(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),
        rudder_out      : (int16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_rudder),
        throttle_dem    : (int16_t)SpdHgt_Controller->get_throttle_demand(),
        airspeed_estimate : est_airspeed
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wp_distance;
    int16_t target_bearing_cd;
    int16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    float   xtrack_error;
    float   xtrack_error_i;
    float   airspeed_error;
    int32_t target_lat;
    int32_t target_lng;
    int32_t target_alt;
    int32_t target_airspeed;
};

// Write a navigation tuning packet
void Plane::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        wp_distance         : auto_state.wp_distance,
        target_bearing_cd   : (int16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (int16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)altitude_error_cm,
        xtrack_error        : nav_controller->crosstrack_error(),
        xtrack_error_i      : nav_controller->crosstrack_error_integrator(),
        airspeed_error      : airspeed_error,
        target_lat          : next_WP_loc.lat,
        target_lng          : next_WP_loc.lng,
        target_alt          : next_WP_loc.alt,
        target_airspeed     : target_airspeed_cm,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Status {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t is_flying;
    float is_flying_probability;
    uint8_t armed;
    uint8_t safety;
    bool is_crashed;
    bool is_still;
    uint8_t stage;
    bool impact;
};

void Plane::Log_Write_Status()
{
    struct log_Status pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STATUS_MSG)
        ,time_us   : AP_HAL::micros64()
        ,is_flying   : is_flying()
        ,is_flying_probability : isFlyingProbability
        ,armed       : hal.util->get_soft_armed()
        ,safety      : static_cast<uint8_t>(hal.util->safety_switch_state())
        ,is_crashed  : crash_state.is_crashed
        ,is_still    : AP::ins().is_still()
        ,stage       : static_cast<uint8_t>(flight_stage)
        ,impact      : crash_state.impact_detected
        };

    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Sonar {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float distance;
    float voltage;
    uint8_t count;
    float correction;
};

// Write a sonar packet.  Note that RFND log messages are written by
// RangeFinder itself as part of update().
void Plane::Log_Write_Sonar()
{
    uint16_t distance = 0;
    if (rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::RangeFinder_Good) {
        distance = rangefinder.distance_cm_orient(ROTATION_PITCH_270);
    }

    struct log_Sonar pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SONAR_MSG),
        time_us     : AP_HAL::micros64(),
        distance    : (float)distance*0.01f,
        voltage     : rangefinder.voltage_mv_orient(ROTATION_PITCH_270)*0.001f,
        count       : rangefinder_state.in_range_count,
        correction  : rangefinder_state.correction
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AETR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t aileron;
    int16_t elevator;
    int16_t throttle;
    int16_t rudder;
    int16_t flap;
};

void Plane::Log_Write_AETR()
{
    struct log_AETR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AETR_MSG)
        ,time_us  : AP_HAL::micros64()
        ,aileron  : SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)
        ,elevator : SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)
        ,throttle : SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        ,rudder   : SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)
        ,flap     : SRV_Channels::get_output_scaled(SRV_Channel::k_flap_auto)
        };

    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Plane::Log_Write_RC(void)
{
    logger.Write_RCIN();
    logger.Write_RCOUT();
    if (rssi.enabled()) {
        logger.Write_RSSI();
    }
    Log_Write_AETR();
}

void Plane::Log_Write_Guided(void)
{
#if OFFBOARD_GUIDED == ENABLED
    if (control_mode != &mode_guided) {
        return;
    }

    if (guided_state.target_heading_time_ms != 0) {
        logger.Write_PID(LOG_PIDG_MSG, g2.guidedHeading.get_pid_info());
    }

    if (guided_state.target_alt > 0.0f || (guided_state.target_airspeed_cm > 0.001f )) {
        logger.Write("OFG", "TimeUS,Arsp,ArspA,Alt,AltA,AltF,Hdg,HdgA", "QffffBff",
                                               AP_HAL::micros64(),
                                               guided_state.target_airspeed_cm*0.01,
                                               guided_state.target_airspeed_accel,
                                               guided_state.target_alt,
                                               guided_state.target_alt_accel,
                                               guided_state.target_alt_frame,
                                               guided_state.target_heading,
                                               guided_state.target_heading_accel_limit);
    }
#endif // OFFBOARD_GUIDED == ENABLED
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Plane::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "QBH",         "TimeUS,SType,CTot", "s--", "F--" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Qcccchhhf",    "TimeUS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,ThrDem,Aspd", "sdddd---n", "FBBBB---0" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "QfcccfffLLii",  "TimeUS,Dist,TBrg,NavBrg,AltErr,XT,XTi,AspdE,TLat,TLng,TAlt,TAspd", "smddmmmnDUmn", "F0BBB0B0GGBB" },
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "QffBf",   "TimeUS,Dist,Volt,Cnt,Corr", "smv--", "FB0--" },
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "QBBcfff",  "TimeUS,Type,State,Servo,Demanded,Achieved,P", "s---dd-", "F---00-" },
    { LOG_STATUS_MSG, sizeof(log_Status),
      "STAT", "QBfBBBBBB",  "TimeUS,isFlying,isFlyProb,Armed,Safety,Crash,Still,Stage,Hit", "s--------", "F--------" },
    { LOG_QTUN_MSG, sizeof(QuadPlane::log_QControl_Tuning),
      "QTUN", "Qffffffeccf", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DCRt,CRt,TMix", "s----mmmnn-", "F----00000-" },
    { LOG_AOA_SSA_MSG, sizeof(log_AOA_SSA),
      "AOA", "Qff", "TimeUS,AOA,SSA", "sdd", "F00" },

// @LoggerMessage: PIQR,PIQP,PIQY,PIQA
// @Description: QuadPlane Proportional/Integral/Derivative gain values for Roll/Pitch/Yaw/Z
// @Field: TimeUS: Time since system startup
// @Field: Tar: desired value
// @Field: Act: achieved value
// @Field: Err: error between target and achieved
// @Field: P: proportional part of PID
// @Field: I: integral part of PID
// @Field: D: derivative part of PID
// @Field: FF: controller feed-forward portion of response
// @Field: Dmod: scaler applied to D gain to reduce limit cycling
    { LOG_PIQR_MSG, sizeof(log_PID),
      "PIQR", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },
    { LOG_PIQP_MSG, sizeof(log_PID),
      "PIQP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },
    { LOG_PIQY_MSG, sizeof(log_PID),
      "PIQY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },
    { LOG_PIQA_MSG, sizeof(log_PID),
      "PIQA", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },

// @LoggerMessage: PIDG
// @Description: Plane Proportional/Integral/Derivative gain values for Heading when using COMMAND_INT control.
// @Field: TimeUS: Time since system startup
// @Field: Tar: desired value
// @Field: Act: achieved value
// @Field: Err: error between target and achieved
// @Field: P: proportional part of PID
// @Field: I: integral part of PID
// @Field: D: derivative part of PID
// @Field: FF: controller feed-forward portion of response
// @Field: Dmod: scaler applied to D gain to reduce limit cycling
    { LOG_PIDG_MSG, sizeof(log_PID),
      "PIDG", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },

// @LoggerMessage: AETR
// @Description: Normalised pre-mixer control surface outputs
// @Field: TimeUS: Time since system startup
// @Field: Ail: Pre-mixer value for aileron output (between -4500 to 4500)
// @Field: Elev: Pre-mixer value for elevator output (between -4500 to 4500)
// @Field: Thr: Pre-mixer value for throttle output (between -4500 to 4500)
// @Field: Rudd: Pre-mixer value for rudder output (between -4500 to 4500)
// @Field: Flap: Pre-mixer value for flaps output (between -4500 to 4500)
    { LOG_AETR_MSG, sizeof(log_AETR),
      "AETR", "Qhhhhh",  "TimeUS,Ail,Elev,Thr,Rudd,Flap", "s-----", "F-----" },

// @LoggerMessage: OFG
// @Description: OFfboard-Guided - an advanced version of GUIDED for companion computers that includes rate/s.  
// @Field: TimeUS: Time since system startup
// @Field: Arsp:  target airspeed cm
// @Field: ArspA:  target airspeed accel
// @Field: Alt:  target alt
// @Field: AltA: target alt accel
// @Field: AltF: target alt frame
// @Field: Hdg:  target heading
// @Field: HdgA: target heading lim
    { LOG_OFG_MSG, sizeof(log_OFG_Guided),     
      "OFG", "QffffBff",    "TimeUS,Arsp,ArspA,Alt,AltA,AltF,Hdg,HdgA", "s-------", "F-------" }, 

// @LoggerMessage: CMDI
// @Description: Generic CommandInt message logger(CMDI) 
// @Field: TimeUS: Time since system startup
// @Field: CId:  command id
// @Field: TSys: target system
// @Field: TCmp: target component
// @Field: cur:  current
// @Field: cont: autocontinue
// @Field: Prm1: parameter 1
// @Field: Prm2: parameter 2
// @Field: Prm3: parameter 3
// @Field: Prm4: parameter 4
// @Field: Lat: target latitude
// @Field: Lng: target longitude
// @Field: Alt: target altitude
// @Field: F:   frame
    { LOG_CMDI_MSG, sizeof(log_CMDI),     
      "CMDI", "QHBBBBffffiifB",    "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "s---------DUm-", "F---------GGB-" }, 
// these next three are same format as log_CMDI just each a different name for Heading,Speed and Alt COMMAND_INTs
    { LOG_CMDS_MSG, sizeof(log_CMDI),     
      "CMDS", "QHBBBBffffiifB",    "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "s---------DUm-", "F---------GGB-" }, 
    { LOG_CMDA_MSG, sizeof(log_CMDI),     
      "CMDA", "QHBBBBffffiifB",    "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "s---------DUm-", "F---------GGB-" }, 
    { LOG_CMDH_MSG, sizeof(log_CMDI),     
      "CMDH", "QHBBBBffffiifB",    "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "s---------DUm-", "F---------GGB-" }, 

};

void Plane::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

/*
  log a COMMAND_INT message
 */
void Plane::Log_Write_MavCmdI(const mavlink_command_int_t &mav_cmd)
{
    const char *name = "CMDI";
    if (mav_cmd.command == MAV_CMD_GUIDED_CHANGE_SPEED) {
        name = "CMIS";
    } else if (mav_cmd.command == MAV_CMD_GUIDED_CHANGE_ALTITUDE) {
        name = "CMIA";
    } else if (mav_cmd.command == MAV_CMD_GUIDED_CHANGE_HEADING) {
        name = "CMIH";
    }
    logger.Write(name, "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "QHBBBBffffiifB",
                                           AP_HAL::micros64(),
                                           mav_cmd.command,
                                           mav_cmd.target_system,
                                           mav_cmd.target_component,
                                           mav_cmd.current,
                                           mav_cmd.autocontinue,
                                           mav_cmd.param1,
                                           mav_cmd.param2,
                                           mav_cmd.param3,
                                           mav_cmd.param4,
                                           mav_cmd.x,
                                           mav_cmd.y,
                                           mav_cmd.z,
                                           mav_cmd.frame);
}

/*
  initialise logging subsystem
 */
void Plane::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Plane::Log_Write_Attitude(void) {}
void Plane::Log_Write_Fast(void) {}
void Plane::Log_Write_Performance() {}
void Plane::Log_Write_Startup(uint8_t type) {}
void Plane::Log_Write_Control_Tuning() {}
void Plane::Log_Write_Nav_Tuning() {}
void Plane::Log_Write_Status() {}
void Plane::Log_Write_Sonar() {}
void Plane::Log_Write_Guided(void) {}

void Plane::Log_Write_RC(void) {}
void Plane::Log_Write_Vehicle_Startup_Messages() {}

void Plane::log_init(void) {}

#endif // LOGGING_ENABLED
