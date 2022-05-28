#include "urg_sensor.h"
#include <stdio.h>

int main(void)
{
urg_t urg;
long *length_data = NULL;
int ret;
// �^�C���X�^���v�̎擾

// urg_get_distance() �֐��ɕϐ���^���A�^�C���X�^���v���擾����B

const int scan_times = 123;
int length_data_size;
long timestamp;
int i;

// �Z���T���狗���f�[�^���擾����B
ret = urg_start_measurement(&urg, URG_DISTANCE, scan_times, 0);
// \todo check error code

for (i = 0; i < scan_times; ++i) {
    length_data_size = urg_get_distance(&urg, length_data, &timestamp);
    // \todo process length_data array

    // �擾�����^�C���X�^���v���o�͂���
    printf("%ld\n", timestamp);
}
return 0;
}
