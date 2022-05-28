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

#ifndef LASER_PROC_ROS_H
#define LASER_PROC_ROS_H

#include <ros/ros.h>
#include <laser_proc/LaserTransport.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <boost/thread/mutex.hpp>

namespace laser_proc
{ 
  class LaserProcROS
  {
  public:
    LaserProcROS(ros::NodeHandle& n, ros::NodeHandle& pnh);
    
    ~LaserProcROS();

  private:

    void scanCb(const sensor_msgs::MultiEchoLaserScanConstPtr& msg) const;

    /**
     * Callback that is called when there is a new subscriber.
     * 
     * Will not subscribe until we have a subscriber for our LaserScans (lazy subscribing).
     * 
     */
    void connectCb(const ros::SingleSubscriberPublisher& pub);

    /**
     * Callback called when a subscriber unsubscribes.
     * 
     * If all current subscribers of our LaserScans stop listening, stop subscribing (lazy subscribing).
     * 
     */
    void disconnectCb(const ros::SingleSubscriberPublisher& pub);
    
    ros::NodeHandle nh_; ///< Nodehandle used to subscribe in the connectCb.
    laser_proc::LaserPublisher pub_; ///< Publisher
    ros::Subscriber sub_; ///< Multi echo subscriber
    
    boost::mutex connect_mutex_; ///< Prevents the connectCb and disconnectCb from being called until everything is initialized.
  };
  
  
}; // depthimage_to_laserscan

#endif