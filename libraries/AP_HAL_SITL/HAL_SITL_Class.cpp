#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "Scheduler.h"
#include "AnalogIn.h"
#include "UARTDriver.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "SITL_State.h"
#include "Util.h"
#include "DSP.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Logger/AP_Logger.h>

#include <SITL/Serialize.h>

// as far to the bottom as we can to leave the other headers in their existing order
#include <../../ArduCopter/Copter.h> // for boost 

using namespace HALSITL;

static Storage sitlStorage;
static SITL_State sitlState;
static Scheduler sitlScheduler(&sitlState);
static RCInput  sitlRCInput(&sitlState);
static RCOutput sitlRCOutput(&sitlState);
static AnalogIn sitlAnalogIn(&sitlState);
static GPIO sitlGPIO(&sitlState);
static DSP dspDriver;


// use the Empty HAL for hardware we don't emulate
static Empty::I2CDeviceManager i2c_mgr_instance;
static Empty::SPIDeviceManager emptySPI;
static Empty::OpticalFlow emptyOpticalFlow;
static Empty::Flash emptyFlash;

static UARTDriver sitlUart0Driver(0, &sitlState);
static UARTDriver sitlUart1Driver(1, &sitlState);
static UARTDriver sitlUart2Driver(2, &sitlState);
static UARTDriver sitlUart3Driver(3, &sitlState);
static UARTDriver sitlUart4Driver(4, &sitlState);
static UARTDriver sitlUart5Driver(5, &sitlState);
static UARTDriver sitlUart6Driver(6, &sitlState);
static UARTDriver sitlUart7Driver(7, &sitlState);

static Util utilInstance(&sitlState);

HAL_SITL::HAL_SITL() :
    AP_HAL::HAL(
        &sitlUart0Driver,   /* uartA */
        &sitlUart1Driver,   /* uartB */
        &sitlUart2Driver,   /* uartC */
        &sitlUart3Driver,   /* uartD */
        &sitlUart4Driver,   /* uartE */
        &sitlUart5Driver,   /* uartF */
        &sitlUart6Driver,   /* uartG */
        &sitlUart7Driver,   /* uartH */
        &i2c_mgr_instance,
        &emptySPI,          /* spi */
        &sitlAnalogIn,      /* analogin */
        &sitlStorage, /* storage */
        &sitlUart0Driver,   /* console */
        &sitlGPIO,          /* gpio */
        &sitlRCInput,       /* rcinput */
        &sitlRCOutput,      /* rcoutput */
        &sitlScheduler,     /* scheduler */
        &utilInstance,      /* util */
        &emptyOpticalFlow,  /* onboard optical flow */
        &emptyFlash,        /* flash driver */
        &dspDriver,         /* dsp driver */
        nullptr),           /* CAN */
    _sitl_state(&sitlState)
{}

static char *new_argv[100];

/*
  save watchdog data
 */
static bool watchdog_save(const uint32_t *data, uint32_t nwords)
{
    int fd = ::open("persistent.dat", O_WRONLY|O_CREAT|O_TRUNC, 0644);
    bool ret = false;
    if (fd != -1) {
        if (::write(fd, data, nwords*4) == (ssize_t)(nwords*4)) {
            ret = true;
        }
        ::close(fd);
    }
    return ret;
}

/*
  load watchdog data
 */
static bool watchdog_load(uint32_t *data, uint32_t nwords)
{
    int fd = ::open("persistent.dat", O_RDONLY, 0644);
    bool ret = false;
    if (fd != -1) {
        ret = (::read(fd, data, nwords*4) == (ssize_t)(nwords*4));
        ::close(fd);
    }
    return ret;
}

/*
  implement watchdoh reset via SIGALRM
 */
static void sig_alrm(int signum)
{
    static char env[] = "SITL_WATCHDOG_RESET=1";
    putenv(env);
    printf("GOT SIGALRM\n");
    execv(new_argv[0], new_argv);
}

void HAL_SITL::exit_signal_handler(int signum)
{
    HALSITL::Scheduler::_should_exit = true;
}

void HAL_SITL::setup_signal_handlers() const
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = HAL_SITL::exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
}

/*
  fill 8k of stack with NaN. This allows us to find uses of
  uninitialised memory without valgrind
 */
static void fill_stack_nan(void)
{
    float stk[2048];
    fill_nanf(stk, ARRAY_SIZE(stk));
}

void HAL_SITL::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    _sitl_state->init(argc, argv);

    scheduler->init();
    uartA->begin(115200);

    rcin->init();
    rcout->init();

    // spi->init();
    analogin->init();

    if (getenv("SITL_WATCHDOG_RESET")) {
        INTERNAL_ERROR(AP_InternalError::error_t::watchdog_reset);
        if (watchdog_load((uint32_t *)&utilInstance.persistent_data, (sizeof(utilInstance.persistent_data)+3)/4)) {
            uartA->printf("Loaded watchdog data");
            utilInstance.last_persistent_data = utilInstance.persistent_data;
        }
    }

    // form a new argv, removing problem parameters. This is used for reboot
    uint8_t new_argv_offset = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(new_argv) && i<argc; i++) {
        if (!strcmp(argv[i], "-w")) {
            // don't wipe params on reboot
            continue;
        }
        new_argv[new_argv_offset++] = argv[i];
    }
    
    fill_stack_nan();

    callbacks->setup();
    scheduler->system_initialized();

    if (getenv("SITL_WATCHDOG_RESET")) {
        const AP_HAL::Util::PersistentData &pd = util->persistent_data;
        AP::logger().WriteCritical("WDOG", "TimeUS,Task,IErr,IErrCnt,IErrLn,MavMsg,MavCmd,SemLine", "QbIHHHHH",
                                   AP_HAL::micros64(),
                                   pd.scheduler_task,
                                   pd.internal_errors,
                                   pd.internal_error_count,
                                   pd.internal_error_last_line,
                                   pd.last_mavlink_msgid,
                                   pd.last_mavlink_cmd,
                                   pd.semaphore_line);
    }

    bool using_watchdog = AP_BoardConfig::watchdog_enabled();
    if (using_watchdog) {
        signal(SIGALRM, sig_alrm);
        alarm(2);
    }
    setup_signal_handlers();

    uint32_t last_watchdog_save = AP_HAL::millis();

    uint32_t last_persist_save = AP_HAL::millis(); //save
    uint32_t last_persist_load = AP_HAL::millis(); //load
    uint8_t persist_state = 0; // start at 0, move to 1 on save, then to 2 on write

    while (!HALSITL::Scheduler::_should_reboot) {
        if (HALSITL::Scheduler::_should_exit) {
            ::fprintf(stderr, "Exitting\n");
            exit(0);
        }
        fill_stack_nan();
        callbacks->loop();
        HALSITL::Scheduler::_run_io_procs();

        uint32_t now = AP_HAL::millis();
        if (now - last_watchdog_save >= 100 && using_watchdog) {
            // save persistent data every 100ms
            last_watchdog_save = now;
            watchdog_save((uint32_t *)&utilInstance.persistent_data, (sizeof(utilInstance.persistent_data)+3)/4);
        }


        // hardcode 1 persist on startup
        if ((now - last_persist_save >= 500 ) && (persist_state == 0)) {
            // save persistent data every 1000ms
            last_persist_save = now;
            persist_state = 1;

            std::ofstream ofs2("buzz.persist.xml");
            {
                ::printf("Serialised some of sitl to buzz.persist.xml\n");
                boost::archive::xml_oarchive oa(ofs2);
                assert(ofs2.good());
                // write class instance to archive
                oa << BOOST_SERIALIZATION_NVP(_sitl_state);
                oa << BOOST_SERIALIZATION_NVP(callbacks); 
            	// archive and stream closed when destructors are called
            }
        }
        

        // hardcode 1 load 10 secs after save
        if ((now - last_persist_load >= 31000 ) && (persist_state == 1)) {
            // save persistent data every 1000ms
            last_persist_save = now;
            persist_state = 2;

            std::ifstream ifs2("buzz.persist.xml");
            {
                ::printf("DE-Serialised some of sitl to buzz.persist.xml\n");
                boost::archive::xml_iarchive ia(ifs2);
                assert(ifs2.good());
                // read class instance to archive
                ia >> BOOST_SERIALIZATION_NVP(*_sitl_state);
            	// archive and stream closed when destructors are called
            }
        }

        if (using_watchdog) {
            // note that this only works for a speedup of 1
            alarm(2);
        }
    }

    actually_reboot();
}

void HAL_SITL::actually_reboot()
{
    execv(new_argv[0], new_argv);
    AP_HAL::panic("PANIC: REBOOT FAILED: %s", strerror(errno));
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_SITL xhal;
    return xhal;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL

