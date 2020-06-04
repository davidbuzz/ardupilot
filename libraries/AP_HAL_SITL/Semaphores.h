#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_SITL_Namespace.h"
#include <pthread.h>

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
