/*
 * C-callable functions the C++ HAL provides to the Zephyr-side C sources.
 * One declaration, included by the definition and by every caller, so the
 * two cannot drift and -Wmissing-declarations holds - the role
 * AP_HAL_ChibiOS/hwdef/common/stm32_util.h plays for ChibiOS.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Scheduler.cpp: records a fault for the next boot's persistent data, called
   from k_sys_fatal_error_handler() in ap_fault_handler.c. */
void ap_persistent_save_fault(uint16_t line, uint8_t fault_type,
                              uint32_t fault_addr, uint32_t fault_lr,
                              uint32_t fault_icsr);

/* ap_fault_handler.c: hand back the fault record that survived the reset, so
   the HAL can report it over MAVLink. Returns false when nothing is stored.
   Clears the record, so it is reported once per reset. */
bool ap_fault_record_take(unsigned int *reason, uint32_t *pc, uint32_t *lr,
                          uint32_t *cfsr, uint32_t *icsr, uint32_t *thd_prio,
                          uint32_t *count);

/* Util.cpp: renders @SYS/threads.txt into g_ap_sysinfo so it can be read over
   SWD when MAVFTP will not serve it. Called from the io thread. */
void ap_sysinfo_capture(void);

#ifdef __cplusplus
}
#endif
