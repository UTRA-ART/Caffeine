/*!
  \brief URG ƒZƒ“ƒT—p‚Ì•â•ŠÖ”

  \author Satofumi KAMIMURA

  $Id: urg_utils.c,v da778fd816c2 2011/01/05 20:02:06 Satofumi $
*/

#include "urg_c/urg_time.h"

// Portable time function borrowed from ros::Time
void urg_walltime(unsigned long *sec, unsigned long *nsec) 
  {
#ifndef WIN32
#if HAS_CLOCK_GETTIME
    struct  timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    *sec  = start.tv_sec;
    *nsec = start.tv_nsec;
#else
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    sec  = timeofday.tv_sec;
    nsec = timeofday.tv_usec * 1000;
#endif
#else
    // Win32 implementation
    // unless I've missed something obvious, the only way to get high-precision
    // time on Windows is via the QueryPerformanceCounter() call. However,
    // this is somewhat problematic in Windows XP on some processors, especially
    // AMD, because the Windows implementation can freak out when the CPU clocks
    // down to save power. Time can jump or even go backwards. Microsoft has
    // fixed this bug for most systems now, but it can still show up if you have
    // not installed the latest CPU drivers (an oxymoron). They fixed all these
    // problems in Windows Vista, and this API is by far the most accurate that
    // I know of in Windows, so I'll use it here despite all these caveats
    static LARGE_INTEGER cpu_freq, init_cpu_time;
    static uint32_t start_sec = 0;
    static uint32_t start_nsec = 0;
    if ( ( start_sec == 0 ) && ( start_nsec == 0 ) )
      {
        QueryPerformanceFrequency(&cpu_freq);
        if (cpu_freq.QuadPart == 0) {
          throw NoHighPerformanceTimersException();
        }
        QueryPerformanceCounter(&init_cpu_time);
        // compute an offset from the Epoch using the lower-performance timer API
        FILETIME ft;
        GetSystemTimeAsFileTime(&ft);
        LARGE_INTEGER start_li;
        start_li.LowPart = ft.dwLowDateTime;
        start_li.HighPart = ft.dwHighDateTime;
        // why did they choose 1601 as the time zero, instead of 1970?
        // there were no outstanding hard rock bands in 1601.
#ifdef _MSC_VER
        start_li.QuadPart -= 116444736000000000Ui64;
#else
        start_li.QuadPart -= 116444736000000000ULL;
#endif
        start_sec = (uint32_t)(start_li.QuadPart / 10000000); // 100-ns units. odd.
        start_nsec = (start_li.LowPart % 10000000) * 100;
      }
    LARGE_INTEGER cur_time;
    QueryPerformanceCounter(&cur_time);
    LARGE_INTEGER delta_cpu_time;
    delta_cpu_time.QuadPart = cur_time.QuadPart - init_cpu_time.QuadPart;
    // todo: how to handle cpu clock drift. not sure it's a big deal for us.
    // also, think about clock wraparound. seems extremely unlikey, but possible
    double d_delta_cpu_time = delta_cpu_time.QuadPart / (double) cpu_freq.QuadPart;
    uint32_t delta_sec = (uint32_t) floor(d_delta_cpu_time);
    uint32_t delta_nsec = (uint32_t) boost::math::round((d_delta_cpu_time-delta_sec) * 1e9);

    int64_t sec_sum  = (int64_t)start_sec  + (int64_t)delta_sec;
    int64_t nsec_sum = (int64_t)start_nsec + (int64_t)delta_nsec;

    // Throws an exception if we go out of 32-bit range
    normalizeSecNSecUnsigned(sec_sum, nsec_sum);

    sec = sec_sum;
    nsec = nsec_sum;
#endif
  }

void urg_get_walltime(unsigned long long *nsecs){
    unsigned long sec;
    unsigned long nsec;
    urg_walltime(&sec, &nsec);
    *nsecs = (unsigned long long)sec*1000000000ll + (unsigned long long)nsec;
}