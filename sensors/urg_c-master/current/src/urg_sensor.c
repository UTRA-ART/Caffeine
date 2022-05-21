/*!
  \brief URG �Z���T����

  \author Satofumi KAMIMURA

  $Id: urg_sensor.c,v 66816edea765 2011/05/03 06:53:52 satofumi $

  \todo Mx �v�����ɑ��� Mx �R�}���h�𑗐M�����Ƃ��ɁA�K�؂ɓ��삷��悤�ɂ���
*/

#include "urg_c/urg_sensor.h"
#include "urg_c/urg_errno.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(URG_MSC)
#define snprintf _snprintf
#endif


enum {
    URG_FALSE = 0,
    URG_TRUE = 1,

    BUFFER_SIZE = 64 + 2 + 6,

    EXPECTED_END = -1,

    RECEIVE_DATA_TIMEOUT,
    RECEIVE_DATA_COMPLETE,      /*!< �f�[�^�𐳏�Ɏ�M */

    PP_RESPONSE_LINES = 10,
    VV_RESPONSE_LINES = 7,
    II_RESPONSE_LINES = 9,

    MAX_TIMEOUT = 140,
};


static const char NOT_CONNECTED_MESSAGE[] = "not connected.";
static const char RECEIVE_ERROR_MESSAGE[] = "receive error.";


//! �`�F�b�N�T���̌v�Z
static char scip_checksum(const char buffer[], int size)
{
    unsigned char sum = 0x00;
    int i;

    for (i = 0; i < size; ++i) {
        sum += buffer[i];
    }

    // �v�Z�̈Ӗ��� SCIP �d�l�����Q�Ƃ̂���
    return (sum & 0x3f) + 0x30;
}


static int set_errno_and_return(urg_t *urg, int urg_errno)
{
    urg->last_errno = urg_errno;
    return urg_errno;
}


// ��M���������̍s����Ԃ�
static int scip_response(urg_t *urg, const char* command,
                         const int expected_ret[], int timeout,
                         char *receive_buffer, int receive_buffer_max_size)
{
    char *p = receive_buffer;
    char buffer[BUFFER_SIZE];
    int filled_size = 0;
    int line_number = 0;
    int ret = URG_UNKNOWN_ERROR;

    int write_size = (int)strlen(command);
    int n = connection_write(&urg->connection, command, write_size);

    if (n != write_size) {
        return set_errno_and_return(urg, URG_SEND_ERROR);
    }

    if (p) {
        *p = '\0';
    }

    do {
        n = connection_readline(&urg->connection, buffer, BUFFER_SIZE, timeout);
        if (n < 0) {
            return set_errno_and_return(urg, URG_NO_RESPONSE);

        } else if (p && (line_number > 0)
                   && (n < (receive_buffer_max_size - filled_size))) {
            // �G�R�[�o�b�N�͊��S��v�̃`�F�b�N���s�����߁A�i�[���Ȃ�
            memcpy(p, buffer, n);
            p += n;
            *p++ = '\0';
            filled_size += n;
        }

        if (line_number == 0) {
            // �G�R�[�o�b�N�����񂪁A��v���邩���m�F����
            if (strncmp(buffer, command, write_size - 1)) {
                return set_errno_and_return(urg, URG_INVALID_RESPONSE);
            }
        } else if (n > 0) {
            // �G�R�[�o�b�N�ȊO�̍s�̃`�F�b�N�T����]������
            char checksum = buffer[n - 1];
            if ((checksum != scip_checksum(buffer, n - 1)) &&
                (checksum != scip_checksum(buffer, n - 2))) {
                return set_errno_and_return(urg, URG_CHECKSUM_ERROR);
            }
        }

        // �X�e�[�^�X������]�����āA�߂�l�����肷��
        if (line_number == 1) {
            if (n == 1) {
                // SCIP 1.1 �����̏ꍇ�́A���퉞���Ƃ݂Ȃ�
                ret = 0;

            } else if (n != 3) {
                return set_errno_and_return(urg, URG_INVALID_RESPONSE);

            } else {
                int i;
                int actual_ret = strtol(buffer, NULL, 10);
                for (i = 0; expected_ret[i] != EXPECTED_END; ++i) {
                    if (expected_ret[i] == actual_ret) {
                        ret = 0;
                        break;
                    }
                }
            }
        }

        ++line_number;
    } while (n > 0);

    return (ret < 0) ? ret : (line_number - 1);
}


static void ignore_receive_data(urg_t *urg, int timeout)
{
    char buffer[BUFFER_SIZE];
    int n;

    if (urg->is_sending == URG_FALSE) {
        return;
    }

    do {
        n = connection_readline(&urg->connection,
                                buffer, BUFFER_SIZE, timeout);
    } while (n >= 0);

    urg->is_sending = URG_FALSE;
}


static void ignore_receive_data_with_qt(urg_t *urg, int timeout)
{
    if ((urg->is_sending == URG_FALSE) && (urg->is_laser_on == URG_FALSE)) {
        return;
    }

    connection_write(&urg->connection, "QT\n", 3);
    urg->is_sending = URG_TRUE;
    urg->is_laser_on = URG_FALSE;
    ignore_receive_data(urg, timeout);
}


static int change_sensor_baudrate(urg_t *urg,
                                  long current_baudrate, long next_baudrate)
{
    enum { SS_COMMAND_SIZE = 10 };
    char buffer[SS_COMMAND_SIZE];
    int ss_expected[] = { 0, 3, 4, EXPECTED_END };
    int ret;

    if (current_baudrate == next_baudrate) {
        // ���݂̃{�[���[�g�Ɛݒ肷��{�[���[�g���ꏏ�Ȃ�΁A�߂�
        return set_errno_and_return(urg, URG_NO_ERROR);
    }

    // "SS" �R�}���h�Ń{�[���[�g��ύX����
    snprintf(buffer, SS_COMMAND_SIZE, "SS%06ld\n", next_baudrate);
    ret = scip_response(urg, buffer, ss_expected, urg->timeout, NULL, 0);

    // 0F �����̂Ƃ��� Ethernet �p�̃Z���T�Ƃ݂Ȃ��A���퉞����Ԃ�
    if (ret == -15) {
        return set_errno_and_return(urg, URG_NO_ERROR);
    }
    if (ret <= 0) {
        return set_errno_and_return(urg, URG_INVALID_PARAMETER);
    }

    // ���퉞���Ȃ�΁A�z�X�g���̃{�[���[�g��ύX����
    ret = connection_set_baudrate(&urg->connection, next_baudrate);

    // �Z���T���̐ݒ蔽�f��҂��߂ɏ��������ҋ@����
    ignore_receive_data(urg, MAX_TIMEOUT);

    return set_errno_and_return(urg, ret);
}


// �{�[���[�g��ύX���Ȃ���ڑ�����
static int connect_urg_device(urg_t *urg, long baudrate)
{
    long try_baudrate[] = { 19200, 38400, 115200 };
    int try_times = sizeof(try_baudrate) / sizeof(try_baudrate[0]);
    int i;

    // �w�����ꂽ�{�[���[�g����ڑ�����
    for (i = 0; i < try_times; ++i) {
        if (try_baudrate[i] == baudrate) {
            try_baudrate[i] = try_baudrate[0];
            try_baudrate[0] = baudrate;
            break;
        }
    }

    for (i = 0; i < try_times; ++i) {
        enum { RECEIVE_BUFFER_SIZE = 4 };
        int qt_expected[] = { 0, EXPECTED_END };
        char receive_buffer[RECEIVE_BUFFER_SIZE + 1];
        int ret;

        connection_set_baudrate(&urg->connection, try_baudrate[i]);

        // QT �𑗐M���A�������Ԃ���邩�Ń{�[���[�g����v���Ă��邩���m�F����
        ret = scip_response(urg, "QT\n", qt_expected, MAX_TIMEOUT,
                            receive_buffer, RECEIVE_BUFFER_SIZE);
        if (ret > 0) {
            if (!strcmp(receive_buffer, "E")) {
                int scip20_expected[] = { 0, EXPECTED_END };

                // QT �����̍Ō�̉��s��ǂݔ�΂�
                ignore_receive_data(urg, MAX_TIMEOUT);

                // "E" ���Ԃ��ꂽ�ꍇ�́ASCIP 1.1 �Ƃ݂Ȃ� "SCIP2.0" �𑗐M����
                ret = scip_response(urg, "SCIP2.0\n", scip20_expected,
                                    MAX_TIMEOUT, NULL, 0);

                // SCIP2.0 �����̍Ō�̉��s��ǂݔ�΂�
                ignore_receive_data(urg, MAX_TIMEOUT);

                // �{�[���[�g��ύX���Ė߂�
                return change_sensor_baudrate(urg, try_baudrate[i], baudrate);

            } else if (!strcmp(receive_buffer, "0Ee")) {
                int tm2_expected[] = { 0, EXPECTED_END };

                // "0Ee" ���Ԃ��ꂽ�ꍇ�́ATM ���[�h�Ƃ݂Ȃ� "TM2" �𑗐M����
                scip_response(urg, "TM2\n", tm2_expected,
                              MAX_TIMEOUT, NULL, 0);

                // �{�[���[�g��ύX���Ė߂�
                return change_sensor_baudrate(urg, try_baudrate[i], baudrate);
            }
        }

        if (ret <= 0) {
            if (ret == URG_INVALID_RESPONSE) {
                // �ُ�ȃG�R�[�o�b�N�̂Ƃ��́A�����f�[�^��M���Ƃ݂Ȃ���
                // �f�[�^��ǂݔ�΂�
                ignore_receive_data_with_qt(urg, MAX_TIMEOUT);

                // �{�[���[�g��ύX���Ė߂�
                return change_sensor_baudrate(urg, try_baudrate[i], baudrate);

            } else {
                // �������Ȃ��Ƃ��́A�{�[���[�g��ύX���āA�ēx�ڑ����s��
                ignore_receive_data_with_qt(urg, MAX_TIMEOUT);
                continue;
            }
        } else if (!strcmp("00P", receive_buffer)) {

            // �Z���T�ƃz�X�g�̃{�[���[�g��ύX���Ė߂�
            return change_sensor_baudrate(urg, try_baudrate[i], baudrate);
        }
    }

    return set_errno_and_return(urg, URG_NOT_DETECT_BAUDRATE_ERROR);
}


// PP �R�}���h�̉����� urg_t �Ɋi�[����
static int receive_parameter(urg_t *urg)
{
    enum { RECEIVE_BUFFER_SIZE = BUFFER_SIZE * 9, };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    int pp_expected[] = { 0, EXPECTED_END };
    unsigned short received_bits = 0x0000;
    char *p;
    int i;

    int ret = scip_response(urg, "PP\n", pp_expected, MAX_TIMEOUT,
                            receive_buffer, RECEIVE_BUFFER_SIZE);
    if (ret < 0) {
        return ret;
    } else if (ret < PP_RESPONSE_LINES) {
        ignore_receive_data_with_qt(urg, MAX_TIMEOUT);
        return set_errno_and_return(urg, URG_INVALID_RESPONSE);
    }

    p = receive_buffer;
    for (i = 0; i < (ret - 1); ++i) {

        if (!strncmp(p, "DMIN:", 5)) {
            urg->min_distance = strtol(p + 5, NULL, 10);
            received_bits |= 0x0001;

        } else if (!strncmp(p, "DMAX:", 5)) {
            urg->max_distance = strtol(p + 5, NULL, 10);
            received_bits |= 0x0002;

        } else if (!strncmp(p, "ARES:", 5)) {
            urg->area_resolution = strtol(p + 5, NULL, 10);
            received_bits |= 0x0004;

        } else if (!strncmp(p, "AMIN:", 5)) {
            urg->first_data_index = strtol(p + 5, NULL, 10);
            received_bits |= 0x0008;

        } else if (!strncmp(p, "AMAX:", 5)) {
            urg->last_data_index = strtol(p + 5, NULL, 10);
            received_bits |= 0x0010;

        } else if (!strncmp(p, "AFRT:", 5)) {
            urg->front_data_index = strtol(p + 5, NULL, 10);
            received_bits |= 0x0020;

        } else if (!strncmp(p, "SCAN:", 5)) {
            int rpm = strtol(p + 5, NULL, 10);
            // �^�C���A�E�g���Ԃ́A�v�������� 16 �{���x�̒l�ɂ���
            urg->scan_usec = 1000 * 1000 * 60 / rpm;
            urg->timeout = urg->scan_usec >> (10 - 4);
            received_bits |= 0x0040;
        }
        p += strlen(p) + 1;
    }

    // �S�Ẵp�����[�^����M�������m�F
    if (received_bits != 0x007f) {
        return set_errno_and_return(urg, URG_RECEIVE_ERROR);
    }

    urg_set_scanning_parameter(urg,
                               urg->first_data_index - urg->front_data_index,
                               urg->last_data_index - urg->front_data_index,
                               1);

    return set_errno_and_return(urg, URG_NO_ERROR);
}


//! SCIP ������̃f�R�[�h
long urg_scip_decode(const char data[], int size)
{
    const char* p = data;
    const char* last_p = p + size;
    int value = 0;

    while (p < last_p) {
        value <<= 6;
        value &= ~0x3f;
        value |= *p++ - 0x30;
    }
    return value;
}


static int parse_parameter(const char *parameter, int size)
{
    char buffer[5];

    memcpy(buffer, parameter, size);
    buffer[size] = '\0';

    return strtol(buffer, NULL, 10);
}


static urg_measurement_type_t parse_distance_parameter(urg_t *urg,
                                                       const char echoback[])
{
    urg_measurement_type_t ret_type = URG_UNKNOWN;

    urg->received_range_data_byte = URG_COMMUNICATION_3_BYTE;
    if (echoback[1] == 'S') {
        urg->received_range_data_byte = URG_COMMUNICATION_2_BYTE;
        ret_type = URG_DISTANCE;

    } else if (echoback[1] == 'D') {
        if ((echoback[0] == 'G') || (echoback[0] == 'M')) {
            ret_type = URG_DISTANCE;
        } else if ((echoback[0] == 'H') || (echoback[0] == 'N')) {
            ret_type = URG_MULTIECHO;
        }
    } else if (echoback[1] == 'E') {
        if ((echoback[0] == 'G') || (echoback[0] == 'M')) {
            ret_type = URG_DISTANCE_INTENSITY;
        } else if ((echoback[0] == 'H') || (echoback[0] == 'N')) {
            ret_type = URG_MULTIECHO_INTENSITY;
        }
    } else {
        return URG_UNKNOWN;
    }

    // �p�����[�^�̊i�[
    urg->received_first_index = parse_parameter(&echoback[2], 4);
    urg->received_last_index = parse_parameter(&echoback[6], 4);
    urg->received_skip_step = parse_parameter(&echoback[10], 2);

    return ret_type;
}


static urg_measurement_type_t parse_distance_echoback(urg_t *urg,
                                                      const char echoback[])
{
    size_t line_length;
    urg_measurement_type_t ret_type = URG_UNKNOWN;

    if (!strcmp("QT", echoback)) {
        return URG_STOP;
    }

    line_length = strlen(echoback);
    if ((line_length == 12) &&
        ((echoback[0] == 'G') || (echoback[0] == 'H'))) {
        ret_type = parse_distance_parameter(urg, echoback);

    } else if ((line_length == 15) &&
               ((echoback[0] == 'M') || (echoback[0] == 'N'))) {
        ret_type = parse_distance_parameter(urg, echoback);
    }
    return ret_type;
}


static int receive_length_data(urg_t *urg, long length[],
                               unsigned short intensity[],
                               urg_measurement_type_t type, char buffer[])
{
    int n;
    int step_filled = 0;
    int line_filled = 0;
    int multiecho_index = 0;

    int each_size =
        (urg->received_range_data_byte == URG_COMMUNICATION_2_BYTE) ? 2 : 3;
    int data_size = each_size;
    int is_intensity = URG_FALSE;
    int is_multiecho = URG_FALSE;
    int multiecho_max_size = 1;

    if ((type == URG_DISTANCE_INTENSITY) || (type == URG_MULTIECHO_INTENSITY)) {
        data_size *= 2;
        is_intensity = URG_TRUE;
    }
    if ((type == URG_MULTIECHO) || (type == URG_MULTIECHO_INTENSITY)) {
        is_multiecho = URG_TRUE;
        multiecho_max_size = URG_MAX_ECHO;
    }

    do {
        char *p = buffer;
        char *last_p;

        n = connection_readline(&urg->connection,
                                &buffer[line_filled], BUFFER_SIZE - line_filled,
                                urg->timeout);

        if (n > 0) {
            // �`�F�b�N�T���̕]��
            if (buffer[line_filled + n - 1] !=
                scip_checksum(&buffer[line_filled], n - 1)) {
                ignore_receive_data_with_qt(urg, urg->timeout);
                return set_errno_and_return(urg, URG_CHECKSUM_ERROR);
            }
        }

        if (n > 0) {
            line_filled += n - 1;
        }
        last_p = p + line_filled;

        while ((last_p - p) >= data_size) {
            int index;

            if (*p == '&') {
                // �擪������ '&' �������Ƃ��́A�}���`�G�R�[�̃f�[�^�Ƃ݂Ȃ�

                if ((last_p - (p + 1)) < data_size) {
                    // '&' �������āAdata_size ���f�[�^��������Δ�����
                    break;
                }

                --step_filled;
                ++multiecho_index;
                ++p;
                --line_filled;

            } else {
                // ���̃f�[�^
                multiecho_index = 0;
            }

            index = (step_filled * multiecho_max_size) + multiecho_index;

            if (step_filled >
                (urg->received_last_index - urg->received_first_index)) {
                // �f�[�^�����߂���ꍇ�́A�c��̃f�[�^�𖳎����Ė߂�
                ignore_receive_data_with_qt(urg, urg->timeout);
                return set_errno_and_return(urg, URG_RECEIVE_ERROR);
            }


            if (is_multiecho && (multiecho_index == 0)) {
                // �}���`�G�R�[�̃f�[�^�i�[����_�~�[�f�[�^�Ŗ��߂�
                int i;
                if (length) {
                    for (i = 1; i < multiecho_max_size; ++i) {
                        length[index + i] = 0;
                    }
                }
                if (intensity) {
                    for (i = 1; i < multiecho_max_size; ++i) {
                        intensity[index + i] = 0;
                    }
                }
            }

            // �����f�[�^�̊i�[
            if (length) {
                length[index] = urg_scip_decode(p, 3);
            }
            p += 3;

            // ���x�f�[�^�̊i�[
            if (is_intensity) {
                if (intensity) {
                    intensity[index] = (unsigned short)urg_scip_decode(p, 3);
                }
                p += 3;
            }

            ++step_filled;
            line_filled -= data_size;
        }

        // ���ɏ������镶����ޔ�
        memmove(buffer, p, line_filled);
    } while (n > 0);

    return step_filled;
}


//! �����f�[�^�̎擾
static int receive_data(urg_t *urg, long data[], unsigned short intensity[],
                        long *time_stamp, unsigned long long *system_time_stamp)
{
    urg_measurement_type_t type;
    char buffer[BUFFER_SIZE];
    int ret = 0;
    int n;
    int extended_timeout = urg->timeout
        + 2 * (urg->scan_usec * (urg->scanning_skip_scan) / 1000);

    // �G�R�[�o�b�N�̎擾
    n = connection_readline(&urg->connection,
                            buffer, BUFFER_SIZE, extended_timeout);
    if (n <= 0) {
        return set_errno_and_return(urg, URG_NO_RESPONSE);
    }
    // �G�R�[�o�b�N�̉��
    type = parse_distance_echoback(urg, buffer);

    // �����̎擾
    n = connection_readline(&urg->connection,
                            buffer, BUFFER_SIZE, urg->timeout);
    if (n != 3) {
        ignore_receive_data_with_qt(urg, urg->timeout);
        return set_errno_and_return(urg, URG_INVALID_RESPONSE);
    }

    if (buffer[n - 1] != scip_checksum(buffer, n - 1)) {
        // �`�F�b�N�T���̕]��
        ignore_receive_data_with_qt(urg, urg->timeout);
        return set_errno_and_return(urg, URG_CHECKSUM_ERROR);
    }

    if (type == URG_STOP) {
        // QT �����̏ꍇ�ɂ́A�Ō�̉��s��ǂݎ̂āA���퉞���Ƃ��ď�������
        n = connection_readline(&urg->connection,
                                buffer, BUFFER_SIZE, urg->timeout);
        if (n == 0) {
            return 0;
        } else {
            return set_errno_and_return(urg, URG_INVALID_RESPONSE);
        }
    }

    if (urg->specified_scan_times != 1) {
        if (!strncmp(buffer, "00", 2)) {
            // "00" �����̏ꍇ�́A�G�R�[�o�b�N�����Ƃ݂Ȃ��A
            // �Ō�̋�s��ǂݎ̂āA������̃f�[�^��Ԃ�
            n = connection_readline(&urg->connection,
                                    buffer, BUFFER_SIZE, urg->timeout);

            if (n != 0) {
                ignore_receive_data_with_qt(urg, urg->timeout);
                return set_errno_and_return(urg, URG_INVALID_RESPONSE);
            } else {
                return receive_data(urg, data, intensity, time_stamp, system_time_stamp);
            }
        }
    }

    if (((urg->specified_scan_times == 1) && (strncmp(buffer, "00", 2))) ||
        ((urg->specified_scan_times != 1) && (strncmp(buffer, "99", 2)))) {
        if (urg->error_handler) {
            type = urg->error_handler(buffer, urg);
        }

        if (type == URG_UNKNOWN) {
            // Gx, Hx �̂Ƃ��� 00P ���Ԃ��ꂽ�Ƃ����f�[�^
            // Mx, Nx �̂Ƃ��� 99b ���Ԃ��ꂽ�Ƃ����f�[�^
            ignore_receive_data_with_qt(urg, urg->timeout);
            return set_errno_and_return(urg, URG_INVALID_RESPONSE);
        }
    }

    // �^�C���X�^���v�̎擾
    n = connection_readline(&urg->connection,
                            buffer, BUFFER_SIZE, urg->timeout);
    if (n > 0) {
        if (time_stamp) {
            *time_stamp = urg_scip_decode(buffer, 4);
        }
        if (system_time_stamp) {
            urg_get_walltime(system_time_stamp);
        }
    }

    // �f�[�^�̎擾
    switch (type) {
    case URG_DISTANCE:
    case URG_MULTIECHO:
        ret = receive_length_data(urg, data, NULL, type, buffer);
        break;

    case URG_DISTANCE_INTENSITY:
    case URG_MULTIECHO_INTENSITY:
        ret = receive_length_data(urg, data, intensity, type, buffer);
        break;

    case URG_STOP:
    case URG_UNKNOWN:
        ret = 0;
        break;
    }

    // specified_scan_times == 1 �̂Ƃ��� Gx �n�R�}���h���g���邽��
    // �f�[�^�𖾎��I�ɒ�~���Ȃ��Ă悢
    if ((urg->specified_scan_times > 1) && (urg->scanning_remain_times > 0)) {
        if (--urg->scanning_remain_times <= 0) {
            // �f�[�^�̒�~�݂̂��s��
            urg_stop_measurement(urg);
        }
    }
    return ret;
}


int urg_open(urg_t *urg, urg_connection_type_t connection_type,
             const char *device_or_address, long baudrate_or_port)
{
    int ret;
    long baudrate = baudrate_or_port;

    urg->is_active = URG_FALSE;
    urg->is_sending = URG_TRUE;
    urg->last_errno = URG_NOT_CONNECTED;
    urg->timeout = MAX_TIMEOUT;
    urg->scanning_skip_scan = 0;
    urg->error_handler = NULL;

    // �f�o�C�X�ւ̐ڑ�
    ret = connection_open(&urg->connection, connection_type,
                          device_or_address, baudrate_or_port);

    if (ret < 0) {
        switch (connection_type) {
        case URG_SERIAL:
            urg->last_errno = URG_SERIAL_OPEN_ERROR;
            break;

        case URG_ETHERNET:
            urg->last_errno = URG_ETHERNET_OPEN_ERROR;
            break;

        default:
            urg->last_errno = URG_INVALID_RESPONSE;
            break;
        }
        return urg->last_errno;
    }

    // �w�肵���{�[���[�g�� URG �ƒʐM�ł���悤�ɒ���
    if (connection_type == URG_ETHERNET) {
        // Ethernet �̂Ƃ��͉��̒ʐM���x���w�肵�Ă���
        baudrate = 115200;
    }

    if (connect_urg_device(urg, baudrate) != URG_NO_ERROR) {
        return set_errno_and_return(urg, ret);
    }
    urg->is_sending = URG_FALSE;

    // �ϐ��̏�����
    urg->last_errno = URG_NO_ERROR;
    urg->range_data_byte = URG_COMMUNICATION_3_BYTE;
    urg->specified_scan_times = 0;
    urg->scanning_remain_times = 0;
    urg->is_laser_on = URG_FALSE;

    // �p�����[�^�����擾
    ret = receive_parameter(urg);
    if (ret == URG_NO_ERROR) {
        urg->is_active = URG_TRUE;
    }
    return ret;
}


void urg_close(urg_t *urg)
{
    if (urg->is_active) {
        ignore_receive_data_with_qt(urg, urg->timeout);
    }
    connection_close(&urg->connection);
    urg->is_active = URG_FALSE;
}


void urg_set_timeout_msec(urg_t *urg, int msec)
{
    urg->timeout = msec;
}


int urg_start_time_stamp_mode(urg_t *urg)
{
    const int expected[] = { 0, EXPECTED_END };
    int n;

    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    // TM0 �𔭍s����
    n = scip_response(urg, "TM0\n", expected, urg->timeout, NULL, 0);
    if (n <= 0) {
        return set_errno_and_return(urg, URG_INVALID_RESPONSE);
    } else {
        return 0;
    }
}


long urg_time_stamp(urg_t *urg)
{
    const int expected[] = { 0, EXPECTED_END };
    char buffer[BUFFER_SIZE];
    char *p;
    int ret;

    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    ret = scip_response(urg, "TM1\n", expected,
                        urg->timeout, buffer, BUFFER_SIZE);
    if (ret < 0) {
        return ret;
    }

    // buffer ����^�C���X�^���v���擾���A�f�R�[�h���ĕԂ�
    if (strcmp(buffer, "00P")) {
        // �ŏ��̉����� "00P" �łȂ���Ζ߂�
        return set_errno_and_return(urg, URG_RECEIVE_ERROR);
    }
    p = buffer + 4;
    if (strlen(p) != 5) {
        return set_errno_and_return(urg, URG_RECEIVE_ERROR);
    }
    if (p[5] == scip_checksum(p, 4)) {
        return set_errno_and_return(urg, URG_CHECKSUM_ERROR);
    }
    return urg_scip_decode(p, 4);
}


int urg_stop_time_stamp_mode(urg_t *urg)
{
    int expected[] = { 0, EXPECTED_END };
    int n;

    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    // TM2 �𔭍s����
    n = scip_response(urg, "TM2\n", expected, urg->timeout, NULL, 0);
    if (n <= 0) {
        return set_errno_and_return(urg, URG_INVALID_RESPONSE);
    } else {
        return 0;
    }
}


static int send_distance_command(urg_t *urg, int scan_times, int skip_scan,
                                 char single_scan_ch, char continuous_scan_ch,
                                 char scan_type_ch)
{
    char buffer[BUFFER_SIZE];
    int write_size = 0;
    int front_index = urg->front_data_index;
    int n;

    urg->specified_scan_times = (scan_times < 0) ? 0 : scan_times;
    urg->scanning_remain_times = urg->specified_scan_times;
    urg->scanning_skip_scan = (skip_scan < 0) ? 0 : skip_scan;
    if (scan_times >= 100) {
        // �v���񐔂� 99 ���z����ꍇ�́A������̃X�L�������s��
        urg->specified_scan_times = 0;
    }

    if (urg->scanning_remain_times == 1) {
        // ���[�U�������w��
        urg_laser_on(urg);

        write_size = snprintf(buffer, BUFFER_SIZE, "%c%c%04d%04d%02d\n",
                              single_scan_ch, scan_type_ch,
                              urg->scanning_first_step + front_index,
                              urg->scanning_last_step + front_index,
                              urg->scanning_skip_step);
    } else {
        write_size = snprintf(buffer, BUFFER_SIZE, "%c%c%04d%04d%02d%01d%02d\n",
                              continuous_scan_ch, scan_type_ch,
                              urg->scanning_first_step + front_index,
                              urg->scanning_last_step + front_index,
                              urg->scanning_skip_step,
                              skip_scan, urg->specified_scan_times);
        urg->is_sending = URG_TRUE;
    }

    n = connection_write(&urg->connection, buffer, write_size);
    if (n != write_size) {
        return set_errno_and_return(urg, URG_SEND_ERROR);
    }

    return 0;
}


int urg_start_measurement(urg_t *urg, urg_measurement_type_t type,
                          int scan_times, int skip_scan)
{
    char range_byte_ch;
    int ret = 0;

    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    if ((skip_scan < 0) || (skip_scan > 9)) {
        ignore_receive_data_with_qt(urg, urg->timeout);
        return set_errno_and_return(urg, URG_INVALID_PARAMETER);
    }

    // !!! Mx �n, Nx �n�̌v�����̂Ƃ��́AQT �𔭍s���Ă���
    // !!! �v���J�n�R�}���h�𑗐M����悤�ɂ���
    // !!! �������AMD �v������ MD �𔭍s����悤�ɁA�����R�}���h�̏ꍇ��
    // !!! Mx �n, Nx �n�̌v���͏㏑�����邱�Ƃ��ł���悤�ɂ���

    // �w�肳�ꂽ�^�C�v�̃p�P�b�g�𐶐����A���M����
    switch (type) {
    case URG_DISTANCE:
        range_byte_ch =
            (urg->range_data_byte == URG_COMMUNICATION_2_BYTE) ? 'S' : 'D';
        ret = send_distance_command(urg, scan_times, skip_scan,
                                    'G', 'M', range_byte_ch);
        break;

    case URG_DISTANCE_INTENSITY:
        ret = send_distance_command(urg, scan_times, skip_scan,
                                    'G', 'M', 'E');
        break;

    case URG_MULTIECHO:
        ret = send_distance_command(urg, scan_times, skip_scan,
                                    'H', 'N', 'D');
        break;

    case URG_MULTIECHO_INTENSITY:
        ret = send_distance_command(urg, scan_times, skip_scan,
                                    'H', 'N', 'E');
        break;

    case URG_STOP:
    case URG_UNKNOWN:
    default:
        ignore_receive_data_with_qt(urg, urg->timeout);
        urg->last_errno = URG_INVALID_PARAMETER;
        ret = urg->last_errno;
        break;
    }

    return ret;
}


int urg_get_distance(urg_t *urg, long data[], long *time_stamp, unsigned long long *system_time_stamp)
{
    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }
    return receive_data(urg, data, NULL, time_stamp, system_time_stamp);
}


int urg_get_distance_intensity(urg_t *urg,
                               long data[], unsigned short intensity[],
                               long *time_stamp, unsigned long long *system_time_stamp)
{
    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    return receive_data(urg, data, intensity, time_stamp, system_time_stamp);
}


int urg_get_multiecho(urg_t *urg, long data_multi[], long *time_stamp, unsigned long long *system_time_stamp)
{
    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    return receive_data(urg, data_multi, NULL, time_stamp, system_time_stamp);
}


int urg_get_multiecho_intensity(urg_t *urg,
                                long data_multi[],
                                unsigned short intensity_multi[],
                                long *time_stamp, unsigned long long *system_time_stamp)
{
    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    return receive_data(urg, data_multi, intensity_multi, time_stamp, system_time_stamp);
}


int urg_stop_measurement(urg_t *urg)
{
    enum { MAX_READ_TIMES = 3 };
    int ret = URG_INVALID_RESPONSE;
    int n;
    int i;

    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    // QT �𔭍s����
    n = connection_write(&urg->connection, "QT\n", 3);
    if (n != 3) {
        return set_errno_and_return(urg, URG_SEND_ERROR);
    }

    for (i = 0; i < MAX_READ_TIMES; ++i) {
        // QT �̉������Ԃ����܂ŁA�����f�[�^��ǂݎ̂Ă�
        ret = receive_data(urg, NULL, NULL, NULL, NULL);
        if (ret == URG_NO_ERROR) {
            // ���퉞��
            urg->is_laser_on = URG_FALSE;
            urg->is_sending = URG_FALSE;
            return set_errno_and_return(urg, URG_NO_ERROR);
        }
    }
    return ret;
}


int urg_set_scanning_parameter(urg_t *urg, int first_step, int last_step,
                               int skip_step)
{
    // �ݒ�͈̔͊O���w�肵���Ƃ��́A�G���[��Ԃ�
    if (((skip_step < 0) || (skip_step >= 100)) ||
        (first_step > last_step) ||
        (first_step < -urg->front_data_index) ||
        (last_step > (urg->last_data_index - urg->front_data_index))) {
        return set_errno_and_return(urg, URG_SCANNING_PARAMETER_ERROR);
    }

    urg->scanning_first_step = first_step;
    urg->scanning_last_step = last_step;
    urg->scanning_skip_step = skip_step;

    return set_errno_and_return(urg, URG_NO_ERROR);
}


int urg_set_connection_data_size(urg_t *urg,
                                 urg_range_data_byte_t data_byte)
{
    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    if ((data_byte != URG_COMMUNICATION_3_BYTE) ||
        (data_byte != URG_COMMUNICATION_2_BYTE)) {
        return set_errno_and_return(urg, URG_DATA_SIZE_PARAMETER_ERROR);
    }

    urg->range_data_byte = data_byte;

    return set_errno_and_return(urg, URG_NO_ERROR);
}


int urg_laser_on(urg_t *urg)
{
    int expected[] = { 0, 2, EXPECTED_END };
    int ret;

    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    if (urg->is_laser_on != URG_FALSE) {
        // ���Ƀ��[�U���������Ă���Ƃ��́A�R�}���h�𑗐M���Ȃ��悤�ɂ���
        urg->last_errno = 0;
        return urg->last_errno;
    }

    ret = scip_response(urg, "BM\n", expected, urg->timeout, NULL, 0);
    if (ret >= 0) {
        urg->is_laser_on = URG_TRUE;
        ret = 0;
    }
    return ret;
}


int urg_laser_off(urg_t *urg)
{
    return urg_stop_measurement(urg);
}


int urg_reboot(urg_t *urg)
{
    int expected[] = { 0, 1, EXPECTED_END };
    int ret;
    int i;

    if (!urg->is_active) {
        return set_errno_and_return(urg, URG_NOT_CONNECTED);
    }

    // �Q��ڂ� RB ���M��A�ڑ���ؒf����
    for (i = 0; i < 2; ++i) {
        ret = scip_response(urg, "RB\n", expected, urg->timeout, NULL, 0);
        if (ret < 0) {
            return set_errno_and_return(urg, URG_INVALID_RESPONSE);
        }
    }
    urg->is_active = URG_FALSE;
    urg_close(urg);

    urg->last_errno = 0;
    return urg->last_errno;
}


void urg_sleep(urg_t *urg)
{
    enum { RECEIVE_BUFFER_SIZE = 4 };
    int sl_expected[] = { 0, EXPECTED_END };
    char receive_buffer[RECEIVE_BUFFER_SIZE];

    if (urg_stop_measurement(urg) != URG_NO_ERROR) {
        return;
    }

    scip_response(urg, "%SL\n", sl_expected, MAX_TIMEOUT,
                  receive_buffer, RECEIVE_BUFFER_SIZE);
}


void urg_wakeup(urg_t *urg)
{
    urg_stop_measurement(urg);
}


int urg_is_stable(urg_t *urg)
{
    const char *stat = urg_sensor_status(urg);
    return strncmp("Stable", stat, 6) ? 0 : 1;
}


static char *copy_token(char *dest, char *receive_buffer,
                        const char *start_str, const char *end_ch, int lines)
{
    size_t start_str_len = strlen(start_str);
    size_t end_ch_len = strlen(end_ch);
    int i;
    size_t j;

    for (j = 0; j < end_ch_len; ++j) {
        const char *p = receive_buffer;

        for (i = 0; i < lines; ++i) {
            if (!strncmp(p, start_str, start_str_len)) {

                char *last_p = strchr(p + start_str_len, end_ch[j]);
                if (last_p) {
                    *last_p = '\0';
                    memcpy(dest, p + start_str_len,
                           last_p - (p + start_str_len) + 1);
                    return dest;
                }
            }
            p += strlen(p) + 1;
        }
    }
    return NULL;
}


static const char *receive_command_response(urg_t *urg,
                                            char *buffer, int buffer_size,
                                            const char* command,
                                            int response_lines)
{
    const int vv_expected[] = { 0, EXPECTED_END };
    int ret;

    if (!urg->is_active) {
        return NOT_CONNECTED_MESSAGE;
    }

    ret = scip_response(urg, command, vv_expected, urg->timeout,
                        buffer, buffer_size);
    if (ret < response_lines) {
        return RECEIVE_ERROR_MESSAGE;
    }

    return NULL;
}


const char *urg_sensor_product_type(urg_t *urg)
{
    enum {
        RECEIVE_BUFFER_SIZE = BUFFER_SIZE * VV_RESPONSE_LINES,
    };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    const char *ret;
    char *p;

    ret = receive_command_response(urg, receive_buffer, RECEIVE_BUFFER_SIZE,
                                   "VV\n", VV_RESPONSE_LINES);
    if (ret) {
        return ret;
    }

    p = copy_token(urg->return_buffer,
                   receive_buffer, "PROD:", ";", VV_RESPONSE_LINES);
    return (p) ? p : RECEIVE_ERROR_MESSAGE;
}


const char *urg_sensor_serial_id(urg_t *urg)
{
    enum {
        RECEIVE_BUFFER_SIZE = BUFFER_SIZE * VV_RESPONSE_LINES,
    };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    const char *ret;
    char *p;

    ret = receive_command_response(urg, receive_buffer, RECEIVE_BUFFER_SIZE,
                                   "VV\n", VV_RESPONSE_LINES);
    if (ret) {
        return ret;
    }

    p = copy_token(urg->return_buffer,
                   receive_buffer, "SERI:", ";", VV_RESPONSE_LINES);
    return (p) ? p : RECEIVE_ERROR_MESSAGE;
}

const char *urg_sensor_vendor(urg_t *urg){
    enum {
        RECEIVE_BUFFER_SIZE = BUFFER_SIZE * VV_RESPONSE_LINES,
    };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    const char *ret;
    char *p;

    if (!urg->is_active) {
        return NOT_CONNECTED_MESSAGE;
    }

    ret = receive_command_response(urg, receive_buffer, RECEIVE_BUFFER_SIZE,
                                   "VV\n", VV_RESPONSE_LINES);
    if (ret) {
        return ret;
    }

    p = copy_token(urg->return_buffer,
                   receive_buffer, "VEND:", ";", VV_RESPONSE_LINES);
    return (p) ? p : RECEIVE_ERROR_MESSAGE;
}

const char *urg_sensor_firmware_version(urg_t *urg)
{
    enum {
        RECEIVE_BUFFER_SIZE = BUFFER_SIZE * VV_RESPONSE_LINES,
    };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    const char *ret;
    char *p;

    if (!urg->is_active) {
        return NOT_CONNECTED_MESSAGE;
    }

    ret = receive_command_response(urg, receive_buffer, RECEIVE_BUFFER_SIZE,
                                   "VV\n", VV_RESPONSE_LINES);
    if (ret) {
        return ret;
    }

    p = copy_token(urg->return_buffer,
                   receive_buffer, "FIRM:", "(", VV_RESPONSE_LINES);
    return (p) ? p : RECEIVE_ERROR_MESSAGE;
}

const char *urg_sensor_firmware_date(urg_t *urg)
{
    enum {
        RECEIVE_BUFFER_SIZE = BUFFER_SIZE * VV_RESPONSE_LINES,
    };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    const char *ret;
    char *p;

    if (!urg->is_active) {
        return NOT_CONNECTED_MESSAGE;
    }

    // Get the firmware version and append a '(', this will be what's before the date
    char firmware_version[50];
    strcpy(firmware_version, urg_sensor_firmware_version(urg));
    strcat(firmware_version, "(");

    ret = receive_command_response(urg, receive_buffer, RECEIVE_BUFFER_SIZE,
                                   "VV\n", VV_RESPONSE_LINES);
    if (ret) {
        return ret;
    }

    // Strip out the actual date from between the '(' and ')'
    char *date;
    p = copy_token(urg->return_buffer,
               receive_buffer, "FIRM:", ";", VV_RESPONSE_LINES);
    date = copy_token(urg->return_buffer, p, firmware_version, ")", 1);
    return (date) ? date : RECEIVE_ERROR_MESSAGE;
}

const char *urg_sensor_protocol_version(urg_t *urg)
{
    enum {
        RECEIVE_BUFFER_SIZE = BUFFER_SIZE * VV_RESPONSE_LINES,
    };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    const char *ret;
    char *p;

    if (!urg->is_active) {
        return NOT_CONNECTED_MESSAGE;
    }

    ret = receive_command_response(urg, receive_buffer, RECEIVE_BUFFER_SIZE,
                                   "VV\n", VV_RESPONSE_LINES);
    if (ret) {
        return ret;
    }

    p = copy_token(urg->return_buffer,
                   receive_buffer, "PROT:", ";", VV_RESPONSE_LINES);
    return (p) ? p : RECEIVE_ERROR_MESSAGE;
}


const char *urg_sensor_status(urg_t *urg)
{
    enum {
        RECEIVE_BUFFER_SIZE = BUFFER_SIZE * II_RESPONSE_LINES,
    };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    const char *ret;
    char *p;

    if (!urg->is_active) {
        return NOT_CONNECTED_MESSAGE;
    }

    ret = receive_command_response(urg, receive_buffer, RECEIVE_BUFFER_SIZE,
                                   "II\n", II_RESPONSE_LINES);
    if (ret) {
        return ret;
    }

    p = copy_token(urg->return_buffer,
                   receive_buffer, "STAT:", ";", II_RESPONSE_LINES);
    return (p) ? p : RECEIVE_ERROR_MESSAGE;
}


const char *urg_sensor_state(urg_t *urg)
{
    enum {
        RECEIVE_BUFFER_SIZE = BUFFER_SIZE * II_RESPONSE_LINES,
    };
    char receive_buffer[RECEIVE_BUFFER_SIZE];
    const char *ret;
    char *p;

    if (!urg->is_active) {
        return NOT_CONNECTED_MESSAGE;
    }

    ret = receive_command_response(urg, receive_buffer, RECEIVE_BUFFER_SIZE,
                                   "II\n", II_RESPONSE_LINES);
    if (ret) {
        return ret;
    }

    p = copy_token(urg->return_buffer,
                   receive_buffer, "MESM:", " (", II_RESPONSE_LINES);
    return (p) ? p : RECEIVE_ERROR_MESSAGE;
}


void urg_set_error_handler(urg_t *urg, urg_error_handler handler)
{
    urg->error_handler = handler;
}
