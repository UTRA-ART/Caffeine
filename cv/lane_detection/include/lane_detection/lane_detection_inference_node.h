#ifndef LANE_DETECTION__LANE_DETECTION_INFERENCE_NODE_H
#define LANE_DETECTION__LANE_DETECTION_INFERENCE_NODE_H

#include "image_transport/image_transport.h"
#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

class LaneDetectionInferenceNode
{
  public:
    explicit LaneDetectionInferenceNode(ros::NodeHandle nh);

  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher lane_detection_mask_pub_;

    void imageCallback(const sensor_msgs::ImageConstPtr &image_msg);

    // TODO: should this be a nodelet instead ?
};

#endif // LANE_DETECTION__LANE_DETECTION_INFERENCE_NODE_H
