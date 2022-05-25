/*!
  \~japanese
  \example get_distance_intensity.c �����E���x�f�[�^���擾����

  \author Satofumi KAMIMURA

  $Id: get_distance_intensity.c,v 586c4fa697ef 2011/01/24 08:50:01 Satofumi $
*/

#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"
#include "open_urg_sensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


static void print_data(urg_t *urg, long data[], unsigned short intensity[],
                       int data_n, long time_stamp)
{
#if 1
    int front_index;
    (void)data_n;

    // \~japanese �O���̃f�[�^�݂̂�\��
    front_index = urg_step2index(urg, 0);
    printf("%ld [mm], %d [1], (%ld [msec])\n",
           data[front_index], intensity[front_index], time_stamp);

#else
    (void)urg;

    int i;

    // \~japanese �S�Ẵf�[�^��\��
    printf("# n = %d, time_stamp = %ld\n", data_n, time_stamp);
    for (i = 0; i < data_n; ++i) {
        printf("%d, %ld, %d\n", i, data[i], intensity[i]);
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

	// Distance Intensity reading only (?) on UXM-30LX
	if (!strstr(urg_sensor_product_type(&urg),"UXM-30LX")) {
		fprintf(stderr,"Distance Intensity not supported on %s\n",
				urg_sensor_product_type(&urg));
		return 1;
	}

    max_data_size = urg_max_data_size(&urg);
    data = (long *)malloc(max_data_size * sizeof(data[0]));
    if (!data) {
        perror("urg_max_index()");
        return 1;
    }
    intensity = malloc(max_data_size * sizeof(intensity[0]));
    if (!intensity) {
        perror("urg_max_index()");
        return 1;
    }

    // \~japanese �f�[�^�擾
    urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, CAPTURE_TIMES, 0);
    for (i = 0; i < CAPTURE_TIMES; ++i) {
        n = urg_get_distance_intensity(&urg, data, intensity, &time_stamp, &system_time_stamp);
        if (n <= 0) {
            printf("urg_get_distance_intensity: %s\n", urg_error(&urg));
            free(data);
            urg_close(&urg);
            return 1;
        }
        print_data(&urg, data, intensity, n, time_stamp);
    }

    // \~japanese �ؒf
    free(intensity);
    free(data);
    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
