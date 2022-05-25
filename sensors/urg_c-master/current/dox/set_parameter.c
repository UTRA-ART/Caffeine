#include "urg_sensor.h"
#include "urg_utils.h"

int main(void)
{
const char connect_device[] = "/dev/ttyACM0";
const long connect_baudrate = 115200;
urg_t urg;
int first_step;
int last_step;
int skip_step;
int scan_times;
int skip_scan;
int ret;
// �v���p�����[�^�̐ݒ�

// �Z���T�ɑ΂��Đڑ����s���B
// �ڑ����s���ƁA�v���p�����[�^�̐ݒ�͏����������
ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
// \todo check error code

// �v���͈͂��w�肷��
// �Z���T���ʕ����� 90 [deg] �͈͂̃f�[�^�擾���s���A�X�e�b�v�Ԉ������s��Ȃ���
first_step = urg_rad2step(&urg, -45);
last_step = urg_rad2step(&urg, +45);
skip_step = 0;
ret = urg_set_scanning_parameter(&urg, first_step, last_step, skip_step);
// \todo check error code

// �v���񐔂ƌv���̊Ԉ������w�肵�āA�v�����J�n����
// 123 ��̌v�����w�����A�X�L�����̊Ԉ������s��Ȃ���
scan_times = 123;
skip_scan = 0;
ret = urg_start_measurement(&urg, URG_DISTANCE, scan_times, skip_scan);
// \todo check error code
return 0;
}
