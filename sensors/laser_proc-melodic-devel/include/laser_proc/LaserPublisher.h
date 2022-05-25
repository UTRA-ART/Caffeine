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

#ifndef IMAGE_PROC_LASER_PUBLISHER_H
#define IMAGE_PROC_LASER_PUBLISHER_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserEcho.h>

#include <laser_proc/LaserProc.h>

#include <vector>
#include <algorithm>
#include <limits.h>
#include <stdexcept>
#include <sstream>

namespace laser_proc
{

  class LaserTransport;

  class LaserPublisher
  {
  public:
    LaserPublisher() {}

    /*!
     * \brief Returns the number of subscribers that are currently connected to
     * this LaserPublisher.
     *
     * Returns sum of all child publishers.
     */
    uint32_t getNumSubscribers() const;

    /*!
     * \brief Returns the topics of this LaserPublisher.
     */
    std::vector<std::string> getTopics() const;

    /*!
     * \brief Publish a MultiEchoLaserScan on the topics associated with this LaserPublisher.
     */
    void publish(const sensor_msgs::MultiEchoLaserScan& msg) const;

    /*!
     * \brief Publish a MultiEchoLaserScan on the topics associated with this LaserPublisher.
     */
    void publish(const sensor_msgs::MultiEchoLaserScanConstPtr& msg) const;

    /*!
     * \brief Shutdown the advertisements associated with this Publisher.
     */
    void shutdown();

    operator void*() const;
    bool operator< (const LaserPublisher& rhs) const { return impl_ <  rhs.impl_; }
    bool operator!=(const LaserPublisher& rhs) const { return impl_ != rhs.impl_; }
    bool operator==(const LaserPublisher& rhs) const { return impl_ == rhs.impl_; }

  private:
    LaserPublisher(ros::NodeHandle& nh, uint32_t queue_size,
                    const ros::SubscriberStatusCallback& connect_cb,
                    const ros::SubscriberStatusCallback& disconnect_cb,
                    const ros::VoidPtr& tracked_object, bool latch, bool publish_echoes = true);
    
    struct Impl;
    typedef boost::shared_ptr<Impl> ImplPtr;
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    ImplPtr impl_;

    friend class LaserTransport;
  };
  
}; // laser_proc

#endif