/*!
  \~japanese
  \example get_multiecho_intensity.c �����E���x�f�[�^(�}���`�G�R�[)���擾����

  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"
#include "open_urg_sensor.h"
#include <stdio.h>
#include <stdlib.h>


static void print_echo_data(long data[], unsigned short intensity[],
                            int index)
{
    int i;

    // [mm]
    for (i = 0; i < URG_MAX_ECHO; ++i) {
        printf("%ld, ", data[(URG_MAX_ECHO * index) + i]);
    }

    // [1]
    for (i = 0; i < URG_MAX_ECHO; ++i) {
        printf("%d, ", intensity[(URG_MAX_ECHO * index) + i]);
    }
}


// \~japanese �����A���x�̃f�[�^��\������
static void print_data(urg_t *urg, long data[],
                       unsigned short intensity[], int data_n, long time_stamp)
{
#if 1
    int front_index;

    (void)data_n;

    // \~japanese �O���̃f�[�^�݂̂�\��
    front_index = urg_step2index(urg, 0);
    print_echo_data(data, intensity, front_index);
    printf("%ld\n", time_stamp);

#else
    (void)urg;
    int i;

    // \~japanese �S�Ẵf�[�^��\��
    printf("# n = %d, time_stamp = %ld\n", data_n, time_stamp);
    for (i = 0; i < data_n; ++i) {
        print_echo_data(data, intensity, i);
        printf("\n");
    }
#endif
}


int main(int argc, char *argv[])
{
    enum {
        CAPTURE_TIMES = 10,
    };
    urg_t urg;
    int max_data_size;
    long *data = NULL;
    unsigned short *intensity = NULL;
    long time_stamp;
    unsigned long long system_time_stamp;
    int n;
    int i;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    max_data_size = urg_max_data_size(&urg);
    data = (long *)malloc(max_data_size * 3 * sizeof(data[0]));
    intensity = (unsigned short *)malloc(max_data_size * 3 *
                                         sizeof(intensity[0]));

    if (!data) {
        perror("urg_max_index()");
        return 1;
    }

    // \~japanese �f�[�^�擾
    urg_start_measurement(&urg, URG_MULTIECHO_INTENSITY, CAPTURE_TIMES, 0);
    for (i = 0; i < CAPTURE_TIMES; ++i) {
        n = urg_get_multiecho_intensity(&urg, data, intensity, &time_stamp, &system_time_stamp);
        if (n <= 0) {
            printf("urg_get_multiecho_intensity: %s\n", urg_error(&urg));
            free(data);
            free(intensity);
            urg_close(&urg);
            return 1;
        }
        print_data(&urg, data, intensity, n, time_stamp);
    }

    // \~japanese �ؒf
    free(data);
    free(intensity);
    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
