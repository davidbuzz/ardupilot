#include "system.h"

uint16_t WEAK AP_HAL::millis16()
{
    return millis() & 0xFFFF;
}

void WEAK AP_HAL::dump_stack_trace()
{
    // stack dump not available on this platform
}


void WEAK AP_HAL::panic(const char *errormsg, ...) {
    va_list ap;
    va_start(ap, errormsg);
    const size_t MAX_LEN = 1023;
#ifdef __cplusplus
    char msg[MAX_LEN+1] = {0};
#else
    char msg[MAX_LEN+1];
    for (int i = 0; i < MAX_LEN+1; i++)
    {
       msg[i] = 0;
    }
#endif
    vsnprintf(msg, MAX_LEN, errormsg, ap);
    printf("%s\n", msg);
    va_end(ap);
}

