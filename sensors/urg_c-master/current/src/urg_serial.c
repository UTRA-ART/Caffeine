/*!
  \file
  \brief �V���A���ʐM

  \author Satofumi KAMIMURA

  $Id: urg_serial.c,v 0caa22c18f6b 2010/12/30 03:36:32 Satofumi $
*/

#include "urg_c/urg_serial.h"


enum {
    False = 0,
    True,
};


#if defined(URG_WINDOWS_OS)
#include "urg_serial_windows.c"
#else
#include "urg_serial_linux.c"
#endif


// ���s���ǂ����̔���
static int is_linefeed(const char ch)
{
    return ((ch == '\r') || (ch == '\n')) ? 1 : 0;
}


static void serial_ungetc(urg_serial_t *serial, char ch)
{
    serial->has_last_ch = True;
    serial->last_ch = ch;
}


int serial_readline(urg_serial_t *serial, char *data, int max_size, int timeout)
{
    /* �P�������ǂݏo���ĕ]������ */
    int filled = 0;
    int is_timeout = 0;

    while (filled < max_size) {
        char recv_ch;
        int n = serial_read(serial, &recv_ch, 1, timeout);
        if (n <= 0) {
            is_timeout = 1;
            break;
        } else if (is_linefeed(recv_ch)) {
            break;
        }
        data[filled++] = recv_ch;
    }
    if (filled >= max_size) {
        --filled;
        serial_ungetc(serial, data[filled]);
    }
    data[filled] = '\0';

    if ((filled == 0) && is_timeout) {
        return -1;
    } else {
        //fprintf(stderr, "%s\n", data);
        return filled;
    }
}
