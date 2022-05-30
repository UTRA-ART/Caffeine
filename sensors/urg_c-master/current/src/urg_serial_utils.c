/*!
  \file
  \brief �V���A���p�̕⏕�֐�

  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_c/urg_serial_utils.h"
#include "urg_c/urg_detect_os.h"


#if defined(URG_WINDOWS_OS)
#include "urg_serial_utils_windows.c"
#else
#include "urg_serial_utils_linux.c"
#endif
