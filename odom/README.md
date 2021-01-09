# Odometry #

This folder contains packages that concern Odometry for Caffeine.

## Usage ## 

This package is used if caffeine is spawned via the description package's launch file. Hence, no extra work is required. 

## Relevant Packages & Topics ##

The following internal package is relevant:
- description 

See package.xml and CMakeLists.txt for any external dependancies.

This package only outputs two relevant topics.

### odometry/local ### 

This is a measure of odometry that fuses the odom topic, camera-based odometry (zed/zed_node/odom) and imu data (imu/data) to estimate odometry. 

### odometry/global ###

This is a measure of odometry that fuses local odometry and gps-based odometry. Note, this topic also uses imu data to estimate odometry. 

