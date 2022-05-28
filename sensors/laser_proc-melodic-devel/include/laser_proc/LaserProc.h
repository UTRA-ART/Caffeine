/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#ifndef LASER_PROC_H
#define LASER_PROC_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserEcho.h>

#include <vector>
#include <algorithm>
#include <limits.h>
#include <stdexcept>
#include <sstream>

namespace laser_proc
{ 
  class LaserProc
  {

  public:

    static sensor_msgs::LaserScanPtr getFirstScan(const sensor_msgs::MultiEchoLaserScan& msg);

    static sensor_msgs::LaserScanPtr getLastScan(const sensor_msgs::MultiEchoLaserScan& msg);

    static sensor_msgs::LaserScanPtr getMostIntenseScan(const sensor_msgs::MultiEchoLaserScan& msg);

  private:

    static void fillLaserScan(const sensor_msgs::MultiEchoLaserScan& msg, sensor_msgs::LaserScan& out);

    static size_t getFirstValue(const sensor_msgs::LaserEcho& ranges, float& range);

    static size_t getLastValue(const sensor_msgs::LaserEcho& ranges, float& range);

    static void getMostIntenseValue(const sensor_msgs::LaserEcho& ranges, const sensor_msgs::LaserEcho& intensities, float& range, float& intensity);

  };
  
  
}; // laser_proc

#endif