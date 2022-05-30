/*!
  \~japanese
  \example sync_time_stamp.c �Z���T�� PC �̃^�C���X�^���v�𓯊�����

  \author Satofumi KAMIMURA

  $Id: sync_time_stamp.c,v 799c195d046c 2011/01/14 05:10:38 hokuyo $
*/

#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"
#include "open_urg_sensor.h"
#include <stdio.h>
#if defined(URG_WINDOWS_OS)
#include <time.h>
#else
#include <sys/time.h>
#endif

static int pc_msec_time(void)
{
    static int is_initialized = 0;
#if defined(URG_WINDOWS_OS)
    static clock_t first_clock;
    clock_t current_clock;
#else
    static struct timeval first_time;
    struct timeval current_time;
#endif
    long msec_time;

#if defined(URG_WINDOWS_OS)
    if (!is_initialized) {
        first_clock = clock();
        is_initialized = 1;
    }
    current_clock = clock();
    msec_time = (current_clock - first_clock) * 1000 / CLOCKS_PER_SEC;
#else
    if (!is_initialized) {
        gettimeofday(&first_time, NULL);
	is_initialized = 1;
    }
    gettimeofday(&current_time, NULL);

    msec_time =
        ((current_time.tv_sec - first_time.tv_sec) * 1000) +
        ((current_time.tv_usec - first_time.tv_usec) / 1000);
#endif
    return msec_time;
}


/*!
  \~japanese
  \brief PC �̃^�C���X�^���v�ɕ␳���邽�߂̒l��Ԃ�
*/
static long print_time_stamp(urg_t *urg, long time_stamp_offset)
{
    long sensor_time_stamp;
    long pc_time_stamp;
    long before_pc_time_stamp;
    long after_pc_time_stamp;
    long delay;

    urg_start_time_stamp_mode(urg);

    before_pc_time_stamp = pc_msec_time();
    sensor_time_stamp = urg_time_stamp(urg);
    after_pc_time_stamp = pc_msec_time();
    delay = (after_pc_time_stamp - before_pc_time_stamp) / 2;

    if (sensor_time_stamp < 0) {
        printf("urg_time_stamp: %s\n", urg_error(urg));
        return -1;
    }
    sensor_time_stamp -= time_stamp_offset;

    pc_time_stamp = pc_msec_time();
    urg_stop_time_stamp_mode(urg);

    printf("%ld,\t%ld\n", pc_time_stamp, sensor_time_stamp);

    return sensor_time_stamp - (pc_time_stamp - delay);
}


int main(int argc, char *argv[])
{
    enum {
        TIME_STAMP_PRINT_TIMES = 5,
    };

    urg_t urg;
    long time_stamp_offset;
    int i;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    printf("# pc,\tsensor\n");

    // \~japanese URG �̃^�C���X�^���v�� PC �̃^�C���X�^���v��\��
    time_stamp_offset = print_time_stamp(&urg, 0);

    // \~japanese URG �̕␳��̃^�C���X�^���v�� PC �^�C���X�^���v��\��
    for (i = 0; i < TIME_STAMP_PRINT_TIMES; ++i) {
        print_time_stamp(&urg, time_stamp_offset);
    }

    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
