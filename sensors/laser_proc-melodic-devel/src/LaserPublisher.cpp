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

#include <laser_proc/LaserPublisher.h>

namespace laser_proc {

typedef sensor_msgs::LaserScanPtr (*PublishFunction)(const sensor_msgs::MultiEchoLaserScan& msg);

struct LaserPublisher::Impl
{
  Impl()
    : unadvertised_(false)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !unadvertised_;
  }
  
  void shutdown()
  {
    if (!unadvertised_) {
      unadvertised_ = true;
      for(size_t i = 0; i < pubs_.size(); i++){
        pubs_[i].shutdown();
      }
    }
  }

  ros::Publisher echo_pub_;
  std::vector<ros::Publisher> pubs_;
  std::vector<PublishFunction> functs_;
  bool unadvertised_;
};

///< @TODO Make a static class that creates these
LaserPublisher::LaserPublisher(ros::NodeHandle& nh, uint32_t queue_size,
                                 const ros::SubscriberStatusCallback& connect_cb,
                                 const ros::SubscriberStatusCallback& disconnect_cb,
                                 const ros::VoidPtr& tracked_object, bool latch, bool publish_echoes)
  : impl_(new Impl)
{
  if(publish_echoes){
    impl_->echo_pub_ = nh.advertise<sensor_msgs::MultiEchoLaserScan>("echoes", queue_size, connect_cb, disconnect_cb, tracked_object, latch);
  }
  impl_->pubs_.push_back(nh.advertise<sensor_msgs::LaserScan>("first", queue_size, connect_cb, disconnect_cb, tracked_object, latch));
  impl_->functs_.push_back(laser_proc::LaserProc::getFirstScan);
  impl_->pubs_.push_back(nh.advertise<sensor_msgs::LaserScan>("last", queue_size, connect_cb, disconnect_cb, tracked_object, latch));
  impl_->functs_.push_back(laser_proc::LaserProc::getLastScan);
  impl_->pubs_.push_back(nh.advertise<sensor_msgs::LaserScan>("most_intense", queue_size, connect_cb, disconnect_cb, tracked_object, latch));
  impl_->functs_.push_back(laser_proc::LaserProc::getMostIntenseScan);
}

uint32_t LaserPublisher::getNumSubscribers() const
{
  if (impl_ && impl_->isValid()){
    uint32_t num = impl_->echo_pub_.getNumSubscribers();
    for(size_t i = 0; i < impl_->pubs_.size(); i++){
      num = num + impl_->pubs_[i].getNumSubscribers();
    }
    return num;
  }
  return 0;
}

std::vector<std::string> LaserPublisher::getTopics() const
{
  std::vector<std::string> topics;
  topics.push_back(impl_->echo_pub_.getTopic());
  if (impl_ && impl_->isValid()){
    for(size_t i = 0; i < impl_->pubs_.size(); i++){
      topics.push_back(impl_->pubs_[i].getTopic());
    }
  }
  return topics;
}

void LaserPublisher::publish(const sensor_msgs::MultiEchoLaserScan& msg) const
{
  if (!impl_ || !impl_->isValid()) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::LaserPublisher");
    return;
  }

  // Publish original MultiEchoLaserScan
  if(impl_->echo_pub_){
    if(impl_->echo_pub_.getNumSubscribers() > 0){
      impl_->echo_pub_.publish(msg);
    }
  }
  
  // If needed, publish LaserScans
  for(size_t i = 0; i < impl_->pubs_.size(); i++){
    if(impl_->pubs_[i].getNumSubscribers() > 0){
       try{
        impl_->pubs_[i].publish(impl_->functs_[i](msg));
      } catch(std::runtime_error& e){
        ROS_ERROR_THROTTLE(1.0, "Could not publish to topic %s: %s", impl_->pubs_[i].getTopic().c_str(), e.what());
      }
    }
  }
}

void LaserPublisher::publish(const sensor_msgs::MultiEchoLaserScanConstPtr& msg) const
{
  if (!impl_ || !impl_->isValid()) {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid image_transport::LaserPublisher");
    return;
  }

  // Publish original MultiEchoLaserScan
  if(impl_->echo_pub_){
    impl_->echo_pub_.publish(msg);
  }
  
  // If needed, publish LaserScans
  for(size_t i = 0; i < impl_->pubs_.size(); i++){
    if(impl_->pubs_[i].getNumSubscribers() > 0){
       try{
        impl_->pubs_[i].publish(impl_->functs_[i](*msg));
      } catch(std::runtime_error& e){
        ROS_ERROR_THROTTLE(1.0, "Could not publish to topic %s: %s", impl_->pubs_[i].getTopic().c_str(), e.what());
      }
    }
  }
}

void LaserPublisher::shutdown()
{
  if (impl_) {
    impl_->shutdown();
    impl_.reset();
  }
}

LaserPublisher::operator void*() const
{
  return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0;
}

} //namespace laser_proc

