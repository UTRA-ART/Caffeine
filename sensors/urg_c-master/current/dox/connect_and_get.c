// �V���A���ڑ��ł̃Z���T�Ƃ̐ڑ��Ƌ����f�[�^�̎擾

#include "urg_sensor.h"
#include "urg_utils.h"
#include <stdlib.h>


int main(void)
{
    urg_t urg;
    int ret;
    long *length_data;
    int length_data_size;

    // "COM1" �́A�Z���T���F������Ă���f�o�C�X���ɂ���K�v������
    const char connect_device[] = "COM1";
    const long connect_baudrate = 115200;

    // �Z���T�ɑ΂��Đڑ����s���B
    ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
    // \todo check error code

    // �f�[�^��M�̂��߂̗̈���m�ۂ���
    length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
    // \todo check length_data is not NULL

    // �Z���T���狗���f�[�^���擾����B
    ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
    // \todo check error code

    length_data_size = urg_get_distance(&urg, length_data, NULL);
    // \todo process length_data array

    // �Z���T�Ƃ̐ڑ������B
    urg_close(&urg);

    return 0;
}
