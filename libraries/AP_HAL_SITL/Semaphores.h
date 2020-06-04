#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_SITL_Namespace.h"
#include <pthread.h>

#include <SITL/Serialize.h>

class HALSITL::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;

 template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        //ar & BOOST_SERIALIZATION_NVP(_lock); buzz todo error: ‘union pthread_mutex_t’ has no member named ‘serialize’
    }
protected:
    pthread_mutex_t _lock;
};
