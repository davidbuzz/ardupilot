#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_HAL_SITL_Namespace.h"
#include <sys/time.h>
#include <pthread.h>

#include <SITL/Serialize.h>

#define SITL_SCHEDULER_MAX_TIMER_PROCS 8

/* Scheduler implementation: */
class HALSITL::Scheduler : public AP_HAL::Scheduler {
public:
    explicit Scheduler(SITL_State *sitlState);
    static Scheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<HALSITL::Scheduler*>(scheduler);
    }

    /* AP_HAL::Scheduler methods */

    void init() override;
    void delay(uint16_t ms) override;
    void delay_microseconds(uint16_t us) override;

    void register_timer_process(AP_HAL::MemberProc) override;
    void register_io_process(AP_HAL::MemberProc) override;

    void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;

    bool in_main_thread() const override;
    void system_initialized() override;

    void reboot(bool hold_in_bootloader) override;

    bool interrupts_are_blocked(void) {
        return _nested_atomic_ctr != 0;
    }

    void sitl_begin_atomic() {
        _nested_atomic_ctr++;
    }
    void sitl_end_atomic();

    static void timer_event() {
        _run_timer_procs();
        _run_io_procs();
    }

    uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

    static void _run_io_procs();
    static bool _should_reboot;
    static bool _should_exit;

    /*
      create a new thread
     */
    bool thread_create(AP_HAL::MemberProc, const char *name,
                       uint32_t stack_size, priority_base base, int8_t priority) override;

    void set_in_semaphore_take_wait(bool value) { _in_semaphore_take_wait = value; }
    /*
     * semaphore_wait_hack_required - possibly move time input step
     * forward even if we are currently pretending to be the IO or timer
     * threads.
     */
    // a couple of helper functions to cope with SITL's time stepping
    bool semaphore_wait_hack_required();


    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ::printf("serializing -> %s\n", __PRETTY_FUNCTION__);
        // todo buzz

        //ar & BOOST_SERIALIZATION_NVP(_sitlState);
        ar & BOOST_SERIALIZATION_NVP(_nested_atomic_ctr);
        //ar & BOOST_SERIALIZATION_NVP(_failsafe);

        //ar & BOOST_SERIALIZATION_NVP(_timer_event_missed);
        //ar & BOOST_SERIALIZATION_NVP(_timer_proc);//[SITL_SCHEDULER_MAX_TIMER_PROCS];
        //ar & BOOST_SERIALIZATION_NVP(_io_proc);//[SITL_SCHEDULER_MAX_TIMER_PROCS];
        ar & BOOST_SERIALIZATION_NVP(_num_timer_procs);
        ar & BOOST_SERIALIZATION_NVP(_num_io_procs);
        ar & BOOST_SERIALIZATION_NVP(_in_timer_proc);
        ar & BOOST_SERIALIZATION_NVP(_in_io_proc);

        ar & BOOST_SERIALIZATION_NVP(_in_semaphore_take_wait);
        
        ar & BOOST_SERIALIZATION_NVP(_initialized);
        ar & BOOST_SERIALIZATION_NVP(_stopped_clock_usec);
        ar & BOOST_SERIALIZATION_NVP(_last_io_run);
        //ar & BOOST_SERIALIZATION_NVP(_main_ctx);

        ar & BOOST_SERIALIZATION_NVP(_thread_sem);

        ar & BOOST_SERIALIZATION_NVP(threads);
        //ar & BOOST_SERIALIZATION_NVP(stackfill);

    }
private:
    SITL_State *_sitlState;
    uint8_t _nested_atomic_ctr;
    static AP_HAL::Proc _failsafe;

    static void _run_timer_procs();

    static volatile bool _timer_event_missed;
    static AP_HAL::MemberProc _timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    static AP_HAL::MemberProc _io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    static uint8_t _num_timer_procs;
    static uint8_t _num_io_procs;
    static bool _in_timer_proc;
    static bool _in_io_proc;

    // boolean set by the Semaphore code to indicate it's currently
    // waiting for a take-timeout to occur.
    static bool _in_semaphore_take_wait;

    void stop_clock(uint64_t time_usec) override;

    static void *thread_create_trampoline(void *ctx);
    static void check_thread_stacks(void);
    
    bool _initialized;
    uint64_t _stopped_clock_usec;
    uint64_t _last_io_run;
    pthread_t _main_ctx;

    static HAL_Semaphore _thread_sem;
    struct thread_attr {
        struct thread_attr *next;
        AP_HAL::MemberProc *f;
        pthread_attr_t attr;
        uint32_t stack_size;
        void *stack;
        const uint8_t *stack_min;
        const char *name;

        friend class boost::serialization::access;
        // When the class Archive corresponds to an output archive, the
        // & operator is defined similar to <<.  Likewise, when the class Archive
        // is a type of input archive the & operator is defined similar to >>.
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ::printf("serializing -> %s\n", __PRETTY_FUNCTION__);
            // todo buzz

            ar & BOOST_SERIALIZATION_NVP(next);
            //ar & BOOST_SERIALIZATION_NVP(f);  // error: ‘class Functor<void>’ has no member named ‘serialize’
            //ar & BOOST_SERIALIZATION_NVP(attr); // error: ‘union pthread_attr_t’ has no member named ‘serialize’
            ar & BOOST_SERIALIZATION_NVP(stack_size);
            //ar & BOOST_SERIALIZATION_NVP(stack);
            //ar & BOOST_SERIALIZATION_NVP(stack_min);
            //ar & BOOST_SERIALIZATION_NVP(name);

        }
    };
    static struct thread_attr *threads;
    static const uint8_t stackfill = 0xEB;
};
#endif  // CONFIG_HAL_BOARD
