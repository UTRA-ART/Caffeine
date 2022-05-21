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

#include <laser_proc/LaserProcROS.h>

using namespace laser_proc;
  
LaserProcROS::LaserProcROS(ros::NodeHandle& n, ros::NodeHandle& pnh):nh_(n){
  boost::mutex::scoped_lock lock(connect_mutex_);
  
  // Lazy subscription to multi echo topic
  pub_ = laser_proc::LaserTransport::advertiseLaser(n, 10, boost::bind(&LaserProcROS::connectCb, this, _1), boost::bind(&LaserProcROS::disconnectCb, this, _1), ros::VoidPtr(), false, false);
}

LaserProcROS::~LaserProcROS(){
  sub_.shutdown();
}



void LaserProcROS::scanCb(const sensor_msgs::MultiEchoLaserScanConstPtr& msg) const{
  pub_.publish(msg);
}

void LaserProcROS::connectCb(const ros::SingleSubscriberPublisher& pub){
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (!sub_ && pub_.getNumSubscribers() > 0) {
    ROS_DEBUG("Connecting to multi echo topic.");
    sub_ = nh_.subscribe("echoes", 10, &LaserProcROS::scanCb, this);
  }
}

void LaserProcROS::disconnectCb(const ros::SingleSubscriberPublisher& pub){
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0) {
    ROS_DEBUG("Unsubscribing from multi echo topic.");
    sub_.shutdown();
  }
}