/*!
  \~japanese
  \example sensor_parameter.c �Z���T���̏o��

  \author Satofumi KAMIMURA

  $Id: sensor_parameter.c,v 0caa22c18f6b 2010/12/30 03:36:32 Satofumi $
*/

#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"
#include "open_urg_sensor.h"
#include <stdio.h>


int main(int argc, char *argv[])
{
    urg_t urg;
    int min_step;
    int max_step;
    long min_distance;
    long max_distance;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    printf("Sensor product type: %s\n", urg_sensor_product_type(&urg));
    printf("Sensor firmware version: %s\n", urg_sensor_firmware_version(&urg));
    printf("Sensor serial ID: %s\n", urg_sensor_serial_id(&urg));
    printf("Sensor status: %s\n", urg_sensor_status(&urg));
    printf("Sensor state: %s\n", urg_sensor_state(&urg));

    urg_step_min_max(&urg, &min_step, &max_step);
    printf("step: [%d, %d]\n", min_step, max_step);

    urg_distance_min_max(&urg, &min_distance, &max_distance);
    printf("distance: [%ld, %ld)\n", min_distance, max_distance);

    printf("scan interval: %ld [usec]\n", urg_scan_usec(&urg));
    printf("sensor data size: %d\n", urg_max_data_size(&urg));

    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
