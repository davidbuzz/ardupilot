#pragma once

#include <time.h> //newlib's time.h hopefully has most of what we need

#include <sys/types.h> // for clockid_t



#ifdef BUILT_WITH_CMAKE
// enough forard declarations for gmtime_r() to work - ardupilot uses this heavily
struct tm {
   int tm_sec;         /* seconds,  range 0 to 59          */
   int tm_min;         /* minutes, range 0 to 59           */
   int tm_hour;        /* hours, range 0 to 23             */
   int tm_mday;        /* day of the month, range 1 to 31  */
   int tm_mon;         /* month, range 0 to 11             */
   int tm_year;        /* The number of years since 1900   */
   int tm_wday;        /* day of the week, range 0 to 6    */
   int tm_yday;        /* day in the year, range 0 to 365  */
   int tm_isdst;       /* daylight saving time             */
};

typedef long time_t;
typedef __clockid_t clockid_t;

struct tm *gmtime  ( const time_t *timer );
struct tm *gmtime_r( const time_t *timer, struct tm *buf );
#endif

// replacement for mktime()
time_t ap_mktime(const struct tm *t);


