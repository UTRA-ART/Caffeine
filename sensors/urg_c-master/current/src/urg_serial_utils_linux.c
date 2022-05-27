/*!
  \file
  \brief �V���A���p�̕⏕�֐�

  \author Satofumi KAMIMURA

  $Id: urg_serial_utils_linux.c,v 0caa22c18f6b 2010/12/30 03:36:32 Satofumi $
*/

#include "urg_c/urg_serial_utils.h"
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>


enum {
    MAX_PORTS = 16,
    DEVICE_NAME_SIZE = 255,
};


static char found_ports[MAX_PORTS][DEVICE_NAME_SIZE];
static int found_ports_size = 0;

static const char *search_dir_names[] = {
    "/dev",
    "/dev/usb",
};
static const char *search_base_names[] = {
    "ttyACM",
    "ttyUSB",
    "tty.usbmodem",
};


static void check_base_name(const char* dir_name, const char *file_name)
{
    int n = sizeof(search_base_names) / sizeof(search_base_names[0]);
    int i;

    for (i = 0; i < n; ++i) {
        const char *base_name = search_base_names[i];
        if (!strncmp(base_name, file_name, strlen(base_name))) {
            snprintf(found_ports[found_ports_size], DEVICE_NAME_SIZE,
                     "%s/%s", dir_name, file_name);
            ++found_ports_size;
        }
    }
}


int urg_serial_find_port(void)
{
    int n = sizeof(search_dir_names) / sizeof(search_dir_names[0]);
    int i;

    found_ports_size = 0;
    for (i = 0; i < n; ++i) {
        struct dirent* dir;
        const char *dir_name = search_dir_names[i];
        DIR *dp = opendir(dir_name);
        if (!dp) {
            continue;
        }

        while ((dir = readdir(dp))) {
            check_base_name(dir_name, dir->d_name);
        }
    }
    return found_ports_size;
}


const char *urg_serial_port_name(int index)
{
    if ((index < 0) || (index >= found_ports_size)) {
        return "";
    } else {
        return found_ports[index];
    }
}


int urg_serial_is_urg_port(int index)
{
    // Linux �̏ꍇ�A�|�[�g�� URG ���ǂ����͒f��ł��Ȃ�
    // !!! �]�͂�����΁Admesg �Ȃǂ̏o�͂��画�肷��悤�ɂ��Ă��悢
    (void)index;
    return 0;
}
