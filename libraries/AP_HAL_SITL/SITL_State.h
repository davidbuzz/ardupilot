#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "RCInput.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>

#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Terrain/AP_Terrain.h>
#include <SITL/SITL.h>
#include <SITL/SITL_Input.h>
#include <SITL/SIM_Gimbal.h>
#include <SITL/SIM_ADSB.h>
#include <SITL/SIM_Vicon.h>
#include <SITL/SIM_RF_Benewake_TF02.h>
#include <SITL/SIM_RF_Benewake_TF03.h>
#include <SITL/SIM_RF_Benewake_TFmini.h>
#include <SITL/SIM_RF_LightWareSerial.h>
#include <SITL/SIM_RF_Lanbao.h>
#include <SITL/SIM_RF_BLping.h>
#include <SITL/SIM_RF_LeddarOne.h>
#include <SITL/SIM_RF_uLanding_v0.h>
#include <SITL/SIM_RF_uLanding_v1.h>
#include <SITL/SIM_RF_MaxsonarSerialLV.h>
#include <SITL/SIM_RF_Wasp.h>
#include <SITL/SIM_RF_NMEA.h>
#include <SITL/SIM_RF_MAVLink.h>

#include <SITL/SIM_Frsky_D.h>
// #include <SITL/SIM_Frsky_SPort.h>
// #include <SITL/SIM_Frsky_SPortPassthrough.h>

#include <AP_HAL/utility/Socket.h>

// without som sort of boost reference fist, the next ones errror
#include <boost/regex.hpp>

#include <boost/exception/exception.hpp>
#include <boost/current_function.hpp>
#if !defined( BOOST_THROW_EXCEPTION )
#define BOOST_THROW_EXCEPTION(x) ::boost::exception_detail::throw_exception_(x,BOOST_CURRENT_FUNCTION,__FILE__,__LINE__)
#endif

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>




class HAL_SITL;

class HALSITL::SITL_State {
    friend class HALSITL::Scheduler;
    friend class HALSITL::Util;
    friend class HALSITL::GPIO;
public:
    void init(int argc, char * const argv[]);

    enum vehicle_type {
        ArduCopter,
        Rover,
        ArduPlane,
        ArduSub
    };

    int gps_pipe(uint8_t index);
    ssize_t gps_read(int fd, void *buf, size_t count);
    uint16_t pwm_output[SITL_NUM_CHANNELS];
    uint16_t pwm_input[SITL_RC_INPUT_CHANNELS];
    bool output_ready = false;
    bool new_rc_input;
    void loop_hook(void);
    uint16_t base_port(void) const {
        return _base_port;
    }

    // create a file descriptor attached to a virtual device; type of
    // device is given by name parameter
    int sim_fd(const char *name, const char *arg);
    // returns a write file descriptor for a created virtual device
    int sim_fd_write(const char *name);

    bool use_rtscts(void) const {
        return _use_rtscts;
    }
    
    // simulated airspeed, sonar and battery monitor
    uint16_t sonar_pin_value;    // pin 0
    uint16_t airspeed_pin_value; // pin 1
    uint16_t airspeed_2_pin_value; // pin 2
    uint16_t voltage_pin_value;  // pin 13
    uint16_t current_pin_value;  // pin 12
    uint16_t voltage2_pin_value;  // pin 15
    uint16_t current2_pin_value;  // pin 14

    // paths for UART devices
    const char *_uart_path[7] {
        "tcp:0:wait",
        "GPS1",
        "tcp:2",
        "tcp:3",
        "GPS2",
        "tcp:5",
        "tcp:6",
    };

    /* parse a home location string */
    static bool parse_home(const char *home_str,
                           Location &loc,
                           float &yaw_degrees);

private:

    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(output_ready);
        ar & BOOST_SERIALIZATION_NVP(new_rc_input);
        ar & BOOST_SERIALIZATION_NVP(sonar_pin_value);    // pin 0
        ar & BOOST_SERIALIZATION_NVP(airspeed_pin_value); // pin 1
        ar & BOOST_SERIALIZATION_NVP(airspeed_2_pin_value); // pin 2
        ar & BOOST_SERIALIZATION_NVP(voltage_pin_value);  // pin 13
        ar & BOOST_SERIALIZATION_NVP(current_pin_value);  // pin 12
        ar & BOOST_SERIALIZATION_NVP(voltage2_pin_value);  // pin 15
        ar & BOOST_SERIALIZATION_NVP(current2_pin_value);  // pin 14

        //ar & _uart_path; // char * ?

        //ar & _gps_data; // ‘struct HALSITL::SITL_State::gps_data’ has no member named ‘serialize’

        ar & BOOST_SERIALIZATION_NVP(_vehicle);
        ar &  BOOST_SERIALIZATION_NVP(_framerate);
        ar &  BOOST_SERIALIZATION_NVP(_instance);
        ar &  BOOST_SERIALIZATION_NVP(_base_port);
        ar &  BOOST_SERIALIZATION_NVP(_parent_pid);
        ar &  BOOST_SERIALIZATION_NVP(_update_count);

        
        // ar & _barometer;//‘class AP_Baro’ has no member named ‘serialize’
        //ar & _ins;
        //ar & _scheduler;
        //ar & _compass; //‘class Compass’ has no member named ‘serialize’
        //ar & _terrain;
       
        //ar &  _sitl_rc_in; // cant serialize SocketAPM 
        // ar & _sitl;  //‘class SITL::SITL’ has no member named ‘serialize’
        ar &  BOOST_SERIALIZATION_NVP(_rcin_port);
        ar &  BOOST_SERIALIZATION_NVP(_fg_view_port);
        ar &  BOOST_SERIALIZATION_NVP(_irlock_port);
        ar &  BOOST_SERIALIZATION_NVP(_current);

        ar &  BOOST_SERIALIZATION_NVP(_synthetic_clock_mode);

        ar &  BOOST_SERIALIZATION_NVP(_use_rtscts);
        ar &  BOOST_SERIALIZATION_NVP(_use_fg_view);
        
        //ar & _fg_address; // char *

        //ar &  mag_buffer_length; //static const
        //ar &  wind_buffer_length; // static const

        ar &  BOOST_SERIALIZATION_NVP(store_index_mag);
        ar &  BOOST_SERIALIZATION_NVP(last_store_time_mag);
      //  ar & buffer_mag;  //‘class VectorN<HALSITL::SITL_State::readings_mag, 250>’ has no member named ‘serialize’
        ar &  BOOST_SERIALIZATION_NVP(time_delta_mag);
        ar &  BOOST_SERIALIZATION_NVP(delayed_time_mag);

        ar &  BOOST_SERIALIZATION_NVP(store_index_wind);
        ar &  BOOST_SERIALIZATION_NVP(last_store_time_wind);
        //ar & buffer_wind;
       // ar & buffer_wind_2;//‘class VectorN<HALSITL::SITL_State::readings_wind, 50>’ has no member named ‘serialize’
        ar &  BOOST_SERIALIZATION_NVP(time_delta_wind);
        ar &  BOOST_SERIALIZATION_NVP(delayed_time_wind);
        ar &  BOOST_SERIALIZATION_NVP(wind_start_delay_micros);

        //ar & sitl_model; scary polymorphic pointer that causes boost to segfault. 
//boost::archive::detail::save_pointer_type<boost::archive::text_oarchive>::polymorphic::save<SITL::Aircraft> (ar=..., t=...)


        ar & BOOST_SERIALIZATION_NVP(enable_gimbal);
        //ar & gimbal;
        //ar & adsb;
        //ar & lightwareserial;
        //ar & frsky_d;
        //ar & fg_socket; // cant serialize SocketAPM 

        //ar & defaults_path; //char *
        //ar & _home_str;  // char*

    }

    void _parse_command_line(int argc, char * const argv[]);
    void _set_param_default(const char *parm);
    void _usage(void);
    void _sitl_setup(const char *home_str);
    void _setup_fdm(void);
    void _setup_timer(void);
    void _setup_adc(void);

    void set_height_agl(void);
    void _update_rangefinder(float range_value);
    void _set_signal_handlers(void) const;

    struct gps_data {
        double latitude;
        double longitude;
        float altitude;
        double speedN;
        double speedE;
        double speedD;
        double yaw;
        bool have_lock;
    };

#define MAX_GPS_DELAY 100
    gps_data _gps_data[2][MAX_GPS_DELAY];

    bool _gps_has_basestation_position;
    gps_data _gps_basestation_data;
    void _gps_write(const uint8_t *p, uint16_t size, uint8_t instance);
    void _gps_send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size, uint8_t instance);
    void _update_gps_ubx(const struct gps_data *d, uint8_t instance);
    void _update_gps_mtk(const struct gps_data *d, uint8_t instance);
    void _update_gps_mtk16(const struct gps_data *d, uint8_t instance);
    void _update_gps_mtk19(const struct gps_data *d, uint8_t instance);
    uint16_t _gps_nmea_checksum(const char *s);
    void _gps_nmea_printf(uint8_t instance, const char *fmt, ...);
    void _update_gps_nmea(const struct gps_data *d, uint8_t instance);
    void _sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload, uint8_t instance);
    void _update_gps_sbp(const struct gps_data *d, uint8_t instance);
    void _update_gps_sbp2(const struct gps_data *d, uint8_t instance);
    void _update_gps_file(uint8_t instance);
    void _update_gps_nova(const struct gps_data *d, uint8_t instance);
    void _nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen, uint8_t instance);
    uint32_t CRC32Value(uint32_t icrc);
    uint32_t CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc);

    void _update_gps(double latitude, double longitude, float altitude,
                     double speedN, double speedE, double speedD,
                     double yaw, bool have_lock);
    void _update_airspeed(float airspeed);
    void _update_gps_instance(SITL::SITL::GPSType gps_type, const struct gps_data *d, uint8_t instance);
    void _check_rc_input(void);
    bool _read_rc_sitl_input();
    void _fdm_input_local(void);
    void _output_to_flightgear(void);
    void _simulator_servos(struct sitl_input &input);
    void _simulator_output(bool synthetic_clock_mode);
    uint16_t _airspeed_sensor(float airspeed);
    uint16_t _ground_sonar();
    void _fdm_input_step(void);

    void wait_clock(uint64_t wait_time_usec);

    // internal state
    enum vehicle_type _vehicle;
    uint16_t _framerate;
    uint8_t _instance;
    uint16_t _base_port;
    pid_t _parent_pid;
    uint32_t _update_count;

    AP_Baro *_barometer;
    AP_InertialSensor *_ins;
    Scheduler *_scheduler;
    Compass *_compass;
#if AP_TERRAIN_AVAILABLE
    AP_Terrain *_terrain;
#endif

    SocketAPM _sitl_rc_in{true};
    SITL::SITL *_sitl;
    uint16_t _rcin_port;
    uint16_t _fg_view_port;
    uint16_t _irlock_port;
    float _current;

    bool _synthetic_clock_mode;

    bool _use_rtscts;
    bool _use_fg_view;
    
    const char *_fg_address;

    // delay buffer variables
    static const uint8_t mag_buffer_length = 250;
    static const uint8_t wind_buffer_length = 50;

    // magnetometer delay buffer variables
    struct readings_mag {
        uint32_t time;
        Vector3f data;
    };
    uint8_t store_index_mag;
    uint32_t last_store_time_mag;
    VectorN<readings_mag,mag_buffer_length> buffer_mag;
    uint32_t time_delta_mag;
    uint32_t delayed_time_mag;

    // airspeed sensor delay buffer variables
    struct readings_wind {
        uint32_t time;
        float data;
    };
    uint8_t store_index_wind;
    uint32_t last_store_time_wind;
    VectorN<readings_wind,wind_buffer_length> buffer_wind;
    VectorN<readings_wind,wind_buffer_length> buffer_wind_2;
    uint32_t time_delta_wind;
    uint32_t delayed_time_wind;
    uint32_t wind_start_delay_micros;

    // internal SITL model
    SITL::Aircraft *sitl_model;

    // simulated gimbal
    bool enable_gimbal;
    SITL::Gimbal *gimbal;

    // simulated ADSb
    SITL::ADSB *adsb;

    // simulated vicon system:
    SITL::Vicon *vicon;

    // simulated Benewake tf02 rangefinder:
    SITL::RF_Benewake_TF02 *benewake_tf02;
    // simulated Benewake tf03 rangefinder:
    SITL::RF_Benewake_TF03 *benewake_tf03;
    // simulated Benewake tfmini rangefinder:
    SITL::RF_Benewake_TFmini *benewake_tfmini;

    // simulated LightWareSerial rangefinder:
    SITL::RF_LightWareSerial *lightwareserial;
    // simulated Lanbao rangefinder:
    SITL::RF_Lanbao *lanbao;
    // simulated BLping rangefinder:
    SITL::RF_BLping *blping;
    // simulated LeddarOne rangefinder:
    SITL::RF_LeddarOne *leddarone;
    // simulated uLanding v0 rangefinder:
    SITL::RF_uLanding_v0 *ulanding_v0;
    // simulated uLanding v1 rangefinder:
    SITL::RF_uLanding_v1 *ulanding_v1;
    // simulated MaxsonarSerialLV rangefinder:
    SITL::RF_MaxsonarSerialLV *maxsonarseriallv;
    // simulated Wasp rangefinder:
    SITL::RF_Wasp *wasp;
    // simulated NMEA rangefinder:
    SITL::RF_NMEA *nmea;
    // simulated MAVLink rangefinder:
    SITL::RF_MAVLink *rf_mavlink;

    // simulated Frsky devices
    SITL::Frsky_D *frsky_d;
    // SITL::Frsky_SPort *frsky_sport;
    // SITL::Frsky_SPortPassthrough *frsky_sportpassthrough;

    // output socket for flightgear viewing
    SocketAPM fg_socket{true};
    
    const char *defaults_path = HAL_PARAM_DEFAULTS_PATH;

    const char *_home_str;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
