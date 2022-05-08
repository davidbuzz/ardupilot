/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"

#define ESP32_SCHEDULER_MAX_TIMER_PROCS 10
#define ESP32_SCHEDULER_MAX_IO_PROCS 10


/* Scheduler implementation: */
class ESP32::Scheduler : public AP_HAL::Scheduler
{

public:
    Scheduler();
    /* AP_HAL::Scheduler methods */
    void     init() override;
    void     set_callbacks(AP_HAL::HAL::Callbacks *cb)
    {
        callbacks = cb;
    };
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;
    bool     in_main_thread() const override;
    // check and set the startup state
    void     set_system_initialized() override;
    bool     is_system_initialized() override;

    void     print_stats(void) ;
    uint16_t get_loop_rate_hz(void);
    AP_Int16 _active_loop_rate_hz;
    AP_Int16 _loop_rate_hz;

    static void thread_create_trampoline(void *ctx);
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;

/*
before:  cpu is mostly idle...

APM_MAIN       	11295279		9%
IDLE           	108898189		90%
IDLE           	106303512		88%
APM_UART       	1185228		<1%
APM_WIFI       	4140027		3%
log_io         	625254		<1%
APM_IO         	1395396		1%
APM_STORAGE    	309057		<1%
tiT            	1963310		1%
APM_TIMER      	864027		<1%
ipc1           	682449		<1%
sys_evt        	1376		<1%
esp_timer      	263610		<1%
wifi           	1920815		1%
Tmr Svc        	15		<1%
ipc0           	742286		<1%

after:  CONFIG_FREERTOS_HZ=200 at least one cpu is closer to 100%
APM_MAIN       	134686198		74%
APM_UART       	10334839		5%
APM_WIFI       	23192287		12%
log_io         	4342472		2%
APM_IO         	7103394		3%
APM_STORAGE    	2184089		1%
IDLE           	137290709		76%
IDLE           	16035044		8%
APM_TIMER      	5138484		2%
tiT            	12726528		7%
sys_evt        	923		<1%
Tmr Svc        	17		<1%
esp_timer      	504347		<1%
wifi           	5396335		2%
ipc1           	851293		<1%
ipc0           	642383		<1%



// with CONFIG_FREERTOS_HZ=400 gets cpu up to 85% and we moved some tasks to other cores, seems to boot ok.
TASK           ABSTIME       PERCENTAGE
/
APM_MAIN       	101931898		84%
APM_UART       	10403211		8%
tiT            	7241598		6%
APM_RCIN       	6806074		5%
log_io         	2761286		2%
APM_IO         	1800989		1%
APM_STORAGE    	1794356		1%
IDLE           	69455261		57%
IDLE           	6629911		5%
APM_TIMER      	7040654		5%
APM_RCOUT      	1338070		1%
sys_evt        	1097		<1%
Tmr Svc        	19		<1%
APM_WIFI       	17263253		14%
esp_timer      	357669		<1%
wifi           	4193745		3%
ipc1           	699632		<1%
ipc0           	655826		<1%
/
STATUS ->Blocked,Ready,Deleted,Suspended
TASK / STATUS /PRIORITY/STACK-HIGH-WATER-MK(REMAINING)/TASKNUM/COREID
/
/
APM_UART       	R	24	384	15	1
APM_WIFI       	R	24	540	19	1
APM_MAIN       	X	24	552	8	0
APM_TIMER      	R	24	628	12	-1
APM_RCIN       	R	15	448	14	-1
log_io         	R	6	980	20	-1
APM_IO         	R	5	1392	16	0
APM_STORAGE    	R	4	344	17	-1
IDLE           	R	0	576	6	1
IDLE           	R	0	588	5	0
tiT            	B	18	1152	9	-1
APM_RCOUT      	B	10	564	13	-1
Tmr Svc        	B	1	1156	7	0
ipc1           	B	24	1076	2	1
esp_timer      	S	22	2844	3	0
wifi           	B	23	3968	11	1
ipc0           	B	24	1136	1	0
sys_evt        	B	20	1432	10	0
/
Heap summary for capabilities 0x00000000:
  At 0x3fcb3c98 len 181096 free 57960 allocated 121380 min_free 6300
    largest_free_block 56320 alloc_blocks 123 free_blocks 4 total_blocks 127
  At 0x3fce0000 len 60980 free 644 allocated 58580 min_free 644
    largest_free_block 0 alloc_blocks 160 free_blocks 0 total_blocks 160
  At 0x3fcf0000 len 32768 free 640 allocated 30372 min_free 628
    largest_free_block 0 alloc_blocks 159 free_blocks 0 total_blocks 159
  At 0x600fe000 len 8192 free 6436 allocated 0 min_free 6436
    largest_free_block 6400 alloc_blocks 0 free_blocks 1 total_blocks 1
  Totals:
    free 65680 allocated 210332 min_free 14008 largest_free_block 56320


*/

// max scheduler priory = 24 according to vTaskList output

    static const int SPI_PRIORITY = 10; //      if your primary imu is spi, this should be above the i2c value, spi is better.
    static const int MAIN_PRIO    = 24; //cpu0: we want schuler running at full tilt.
    static const int I2C_PRIORITY = 5;  //      if your primary imu is i2c, this should be above the spi value, i2c is not preferred.
    static const int TIMER_PRIO   = 24; //      a low priority mere might cause wifi thruput to suffer
    static const int RCIN_PRIO    = 15;
    static const int RCOUT_PRIO   = 10;
    static const int WIFI_PRIO    = 24; //cpu1: 
    static const int UART_PRIO    = 24; //cpu1: a low priority mere might cause wifi thruput to suffer, as wifi gets passed its data frim the uart subsustem in _writebuf/_readbuf
    static const int IO_PRIO      = 5;
    static const int STORAGE_PRIO = 4;

    //_SS vars are number-of-WORDS.        //remaining (stack high water mark):
    static const int TIMER_SS   = 1024+512;//628
    static const int MAIN_SS    = 1024*4;  //632            *3 too small. 0x4037ba21 in panic_abort (details=0x3fce0cf1 "***ERROR*** A stack overflow in task APM_MAIN has been detected.")
    static const int RCIN_SS    = 1024+512;//448 
    static const int RCOUT_SS   = 1024+512;//564
    static const int WIFI_SS    = 1024*2;  //524             with *5, we have <1k free
    static const int UART_SS    = 1024+512;//384             1024 is not enough when SCHEDDEBUG=1, as there's many printf's
    static const int DEVICE_SS  = 4096;    //
    static const int IO_SS      = 4096;    //1392       (APM_IO)
    static const int STORAGE_SS = 2048;    //344        (APM_STORAGE)

private:
    AP_HAL::HAL::Callbacks *callbacks;
    AP_HAL::Proc _failsafe;

    AP_HAL::MemberProc _timer_proc[ESP32_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;

    AP_HAL::MemberProc _io_proc[ESP32_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;

    static bool _initialized;

    int run_timer_state = 0;



    tskTaskControlBlock *_main_task_handle;
    tskTaskControlBlock *_timer_task_handle;
    tskTaskControlBlock *_rcin_task_handle;
    tskTaskControlBlock *_rcout_task_handle;
    tskTaskControlBlock *_uart_task_handle;
    tskTaskControlBlock *_io_task_handle;
    tskTaskControlBlock *test_task_handle;
    tskTaskControlBlock *_storage_task_handle;

    static void _main_thread(void *arg);
    static void _timer_thread(void *arg);
    static void _rcout_thread(void *arg);
    static void _rcin_thread(void *arg);
    static void _uart_thread(void *arg);
    static void _io_thread(void *arg);
    static void _storage_thread(void *arg);

    static void set_position(void* arg);


    static void _print_profile(void* arg);

    static void test_esc(void* arg);

    bool _in_timer_proc;
    void _run_timers();
    Semaphore _timer_sem;

    bool _in_io_proc;
    void _run_io();
    Semaphore _io_sem;
public:
    Semaphore sem;

};
