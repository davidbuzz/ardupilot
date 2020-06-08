#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AccelCalibrator.h"
#include "AP_Vehicle/AP_Vehicle_Type.h"

#include <SITL/Serialize.h>


#define AP_ACCELCAL_MAX_NUM_CLIENTS 4
class GCS_MAVLINK;
class AP_AccelCal_Client;

class AP_AccelCal {
public:
    AP_AccelCal():
    _use_gcs_snoop(true),
    _started(false),
    _saving(false)
    { update_status(); }

    // start all the registered calibrations
    void start(GCS_MAVLINK *gcs);

    // called on calibration cancellation
    void cancel();

    // Run an iteration of all registered calibrations
    void update();

    // get the status of the calibrator server as a whole
    accel_cal_status_t get_status() { return _status; }
    
    // Set vehicle position sent by the GCS
    bool gcs_vehicle_position(float position);

    // interface to the clients for registration
    static void register_client(AP_AccelCal_Client* client);

    void handleMessage(const mavlink_message_t &msg);


    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      ::printf("serializing -> %s\n", __PRETTY_FUNCTION__);
      // buzz todo
      //ar & BOOST_SERIALIZATION_NVP(_gcs); //error: invalid use of incomplete type ‘class GCS_MAVLINK’    BOOST_STATIC_CONSTANT(bool, value = BOOST_IS_ABSTRACT(T));
      ar & BOOST_SERIALIZATION_NVP(_use_gcs_snoop);
      ar & BOOST_SERIALIZATION_NVP(_waiting_for_mavlink_ack);
      ar & BOOST_SERIALIZATION_NVP(_last_position_request_ms);
      ar & BOOST_SERIALIZATION_NVP(_step);
      ar & BOOST_SERIALIZATION_NVP(_status);
      ar & BOOST_SERIALIZATION_NVP(_last_result);

      ar & BOOST_SERIALIZATION_NVP(_num_clients);
      ar & BOOST_SERIALIZATION_NVP(_clients);//[AP_ACCELCAL_MAX_NUM_CLIENTS];

      ar & BOOST_SERIALIZATION_NVP(_started);
      ar & BOOST_SERIALIZATION_NVP(_saving);

      ar & BOOST_SERIALIZATION_NVP(_num_active_calibrators);

    }

private:
    GCS_MAVLINK *_gcs;
    bool _use_gcs_snoop;
    bool _waiting_for_mavlink_ack = false;
    uint32_t _last_position_request_ms;
    uint8_t _step;
    accel_cal_status_t _status;
    accel_cal_status_t _last_result;

    static uint8_t _num_clients;
    static AP_AccelCal_Client* _clients[AP_ACCELCAL_MAX_NUM_CLIENTS];

    // called on calibration success
    void success();

    // called on calibration failure
    void fail();

    // reset all the calibrators to there pre calibration stage so as to make them ready for next calibration request
    void clear();

    // proceed through the collection step for each of the registered calibrators
    void collect_sample();

    // update the state of the Accel calibrator server
    void update_status();

    // checks if no new sample has been received for considerable amount of time
    bool check_for_timeout();

    // check if client's calibrator is active
    bool client_active(uint8_t client_num);

    bool _started;
    bool _saving;

    uint8_t _num_active_calibrators;

    AccelCalibrator* get_calibrator(uint8_t i);
};

class AP_AccelCal_Client {
  //public:
    friend class AP_AccelCal;

    //AP_AccelCal_Client();

    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      ::printf("serializing -> %s\n", __PRETTY_FUNCTION__);
     //   ar & BOOST_SERIALIZATION_NVP(xxx);
    }
  private:
    // getters
    virtual bool _acal_get_saving() { return false; }
    virtual bool _acal_get_ready_to_sample() { return true; }
    virtual bool _acal_get_fail() { return false; }
    virtual AccelCalibrator* _acal_get_calibrator(uint8_t instance) = 0;

    // events
    virtual void _acal_save_calibrations() = 0;
    virtual void _acal_event_success() {};
    virtual void _acal_event_cancellation() {};
    virtual void _acal_event_failure() {};
};
//BOOST_SERIALIZATION_ASSUME_ABSTRACT( AP_AccelCal_Client );

