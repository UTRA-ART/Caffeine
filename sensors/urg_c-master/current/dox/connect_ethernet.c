#include "urg_sensor.h"


int main(void)
{
    urg_t urg;
    int ret;
// �C�[�T�[�l�b�g�ڑ��ł̃Z���T�Ƃ̐ڑ��Ƌ����f�[�^�̎擾

const char connect_address[] = "192.168.0.10";
const long connect_port = 10940;

// �Z���T�ɑ΂��Đڑ����s���B
ret = urg_open(&urg, URG_ETHERNET, connect_address, connect_port);
// \todo check error code
return 0;
}
