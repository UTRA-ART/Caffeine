^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urg_c
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.405 (2018-08-07)
--------------------
* pass *pointer to* system timestamp, sync issue on quit
* Adding new maintainer.
* Contributors: Tony Baltovski, knickels

1.0.404 (2014-04-28)
--------------------
* Merge pull request `#2 <https://github.com/ros-drivers/urg_c/issues/2>`_ from dawonn/master
  Issue `#1 <https://github.com/ros-drivers/urg_c/issues/1>`_ - missing libmath link
* Ubuntu 14.04 Fix - missing libmath link
* Fix build for Android.
* Contributors: Chad Rockey, dawonn

1.0.403 (2013-08-21)
--------------------
* No more Willow Garage email.
* Contributors: Chad Rockey

1.0.402 (2013-04-04)
--------------------
* Missing header for Ubuntu Raring 13.04
* Contributors: chadrockey

1.0.401 (2013-03-03)
--------------------
* Updated version to 1.0.401
* Added files for catkin/bloom to release.
* Added better system timestamp
* Added more information functions.
* Updated includes for urg_c folder.
* Moving header files to be standardized for source builds.
* fix document
* fixed laser off fail when urg_close() has called
* fixed tcpclient_open bug on Linux
* added angle_convert_test.c
* fixed for ethernet sensor
* fixed windows.h include timing
* fixed urg_c-config created
* fixed ethernet open problem
* fixed compile error
* added reported user Adrian Boeing. thanks!
* added releasenotes
* removed rt link script
* fixed for OSX
* added AUTHORS.txt
* fixed plotter's bug
* fixed multiecho data's bug
* fixed urg_tcpclient_open()'s bug
* fixed receive_length data handling
* fixed 'if' condition handling data store
* fixed NULL pointer access bug
* fixed QT after RB
* fixed Gx stop bug
* translated page titles
* fixed urg_sensor_status() function's bug
* fixed buffer size
* remove duplicated file
* fixed doxygen comment
* fixed dependency
* fixed closing time is too long.
* added receive_command_response() internal function.
* added document tag
* sepalated urg_debug.[ch]
* added samples
* fixed download path document.
* urg_sleep(), urg_wakeup, urg_is_stabl() are implemented.
* fixed parameter receive method.
* fixed localhost convert
* fixed localhost ip_address
* added type item
* changed error handler return type
* added urg_scip_decode() function.
* added error handler
* removed temporary image files
* fixed OpenGL 1 support
* fixed html mainpage
* added sample compile check
* translated some files.
* translated mainpage.dox
* fixed document.
* added timestamp tutorial
* added images
* fixed indent
* added doxygen comment
* added document.
* added tutorial samples
* fixed dox comment
* mainpage is created.
* fixed document mainpage layout
* added usage document.
* adjusted mainpage.dox
* modify how to build sample with Visual Studio on Readme_ja.txt
* added install dox document
* added install document
* fixed laser_off handling
* fixed connection timeout
* fixed capture_times
* fixed sample's bug
* added visual studio samples.
* added sensor_parameter vc project.
* added visual studio 2005 project files.
* windows compile.bat for winsock2
* fixed for VC++ compile error.
* omit stdbool.h .
* changed UTF-8 -> CP932
* fixed snprintf implementation.
* New directory windowsexe to make windows exe files.
* adjusted for cl.exe compile.
* changed the character encoding SJIS
* added connect timeout function.
* fixed TCP/IP connect timeout
* added CC=gcc
* fixed information funciton's bug
* fixed urg_firmware_version() fail at URG-04LX
* fixed serial connection fail.
* fixed sh scripts
* fixed release package compile failed.
* added comment.
* added impelementation comment.
* fixed winsock close
* fixed Makefile error
* adjusted open routine.
* fixed link libraries
* added mingw install setting
* fixed sort bug.
* fixed urg_serial_utils_windows.c
* implemented urg_serial_utils_windows.c
* implemented urg_serial_utils_linux.c
* added urg_sensor_product_type() function
* fixed scan times bug
* fixed make clean rule
* fixed link directory
* added read/write function
* fixed multiecho sample
* fixed sample bug
* urg_sensor_id() -> urg_sensor_serial_id()
* fixed scan_skip timeout problem
* added extern C
* fixed viewer compile setting
* fixed split script
* fixed sample code's indent
* fixed header indent
* fixed urg_c-config
* urg_c-config addes
* include directory added
* doxygen tag added
* conflict COPYRIGHT and current/COPYRIGHT
* Doxyfile added
* fixed using OS env
* fixed dist rule
* printf debug message removed.
* error output code removed
* key zoom enabled
* Readme.txt modified and COPYRIGHT added.
* urg_tcpclient.h comments
* fixed urg_index2rad()'s bug
* remove debug print
* Merge
* fixed sample args parser
* fixed urg_serial_windows.c compile error.
* added ld_rt.sh
* fixed multiecho intensity bug
* plotter_sdl.c is implemented.
* viewer_sdl debugging
* viewer_sdl.c is implemented.
* fixed struct name
* fixed multiecho sample
* fixed samples for -e option
* adjusted for mingw.
* fixed sh script
* windows socket debugging
* windows tcp debugging
* windows tcp debugging
* fixed mingw compile failed.
* fixed ethernet sample error
* removed -lrt option
* fixed for mingw
* 'urg_tcpclient.c urg_tcpclient.h modified and fixed.'
* urg_tcpclient.c (tcpclient_readline() modified)
* directories reaaranged.
* Readme.txt modified.
* tcpclient debugging.
* urg_tcpclient.c was compiled by mingw gcc.
* Merge
* Merge
* to commit.
* removing urg_ethernet.* files
* tcpclient module for linux.
* added wait enter code
* fixed sample output
* fixed urg_time_stamp()'s bug
* removed implemented todo task comment
* urg_sensor.c:change_sensor_baudrate() is implemented.
* added ethernet option
* fixed sample output
* fixed length data receive misstake
* fixed errno misstake.
* fixed gcc warning
* added device selection ifdef
* Merge
* Merge
* addes multiecho intensity sample
* fixed error handling
* Merge
* Merge
* fixed MD stop
* fixed MD handling
* fixed MD command handling
* multiecho_intensity のテストを追加
* HD command acceptable.
* fixed multiecho parser
* new directory configuration.
* New directory configuration.
* removed debug message
* Merge
* added files using win32
* multiecho function is implemented.
* applied scip_checksum()
* get_distance*() functions were implemented.
* urg_utils.c is implemented.
* RB command is implemented.
* get_distance.c is implemented.
* removed debug code
* added static receive_data_line() function
* fixed infinity loop error
* added parameter test program for URG-04LX
* removed debug message
* fixed indent
* sensor_parameter.cpp is implemented.
* urg_sensor_id() is implemented.
* added debug comment
* added debug comment
* added URG_NOT_DETECT_BAUDRATE_ERROR
* renamed variable
* fixed doxygen comment
* added urg_t variables
* fixed indent
* removed urg_ethernet_t.h
* urg_communication.c is implemented.
* urg_detect_os.h is implemented.
* added urg_detect_os.h
* added windows serial implementetion.
* added serial_test.c
* urg_serial_linux.c is implemented.
* removed urg/Makefile
* changed test function api
* added test case
* adjusted urg directory removed.
* remove urg directory
* add test directory
* add urg_*_t.h
* fixed character-code
* Merge urg_connection
* Merge urg_connection
* Checksum function is implemented
* fixed compile error
* changed connection -> communication
* renamed
* fixed doxygen tag
* added urg_reboot()
* removed urg_connection_utils.h
* added doxygen comment
* added urg_connection.c functions
* implemented some functions.
* add internal functions
* renamed
* added urg_connection.h
* added image
* fix mainpage.dox link
* added urg/sample/get_multiecho_intensity.c
* added doxygen image tag
* fixed urg c API
* changed timestamp -> time_stamp
* added doxygen comment
* added doxygen comment
* added doxygen comment
* adjusted c/urg/*.c files API
* changed C API
* adjusted URG API
* ライブラリの実装方法を追記
* added library API
* added sample programs
* added package files
* added dox files
* add urg manual written in Japanese
* add package files
* Contributors: K. Kimoto, Kunihiro Yasuda, Satofumi KAMIMURA, chadrockey, hokuyo2@free53.hokuyo-aut.co.jp, hokuyo@free53.hokuyo-aut.co.jp, k-yasuda@1433-yasuda3, katsumik, satofumi
