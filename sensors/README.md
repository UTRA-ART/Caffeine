# Instructions ffor adding a new package 

* cd into src dir
* run command 
``` catkin_create_pkg package_name depdency1 dependency2 ... ```
* cd into catkin_ws dir
* run command
``` catkin_make ```
* run command
``` source devel/setup.bash ```
* run command 
``` rosrun package_name file_name.py ```


