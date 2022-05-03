#include <boost/bind/placeholders.hpp>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "lane_detection/lane_detection_inference_node.h"

LaneDetectionInferenceNode::LaneDetectionInferenceNode(ros::NodeHandle nh)
    : nh_{nh}, 
      it_{nh_},
      inferencer_{[&]() {
          nh_.param<std::string>("instance_name", instance_name_, "");
          ROS_INFO("Instance name: %s", instance_name_.c_str());
          nh_.param<std::string>("model_path", model_path_, "");
          ROS_INFO("Model path: %s", model_path_.c_str());
          return LaneDetectionInferencer(instance_name_, model_path_);
      }()},
      image_sub_{it_.subscribe(
          "image", 1,
          boost::bind(&LaneDetectionInferenceNode::imageCallback, this, _1))},
      lane_detection_mask_pub_{it_.advertise("lane_detection_mask", 1)}
{
}

void LaneDetectionInferenceNode::imageCallback(
    const sensor_msgs::ImageConstPtr &image_msg)
{
    cv_bridge::CvImageConstPtr image_msg_ptr;
    try {
        image_msg_ptr =
            cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
        ROS_ERROR("Received cv_bridge error: %s", e.what());
        return;
    }

    lane_detection_mask_pub_.publish(
        cv_bridge::CvImage(
            image_msg->header, sensor_msgs::image_encodings::MONO8,
            inferencer_.getLaneDetectionMask(image_msg_ptr->image))
            .toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detection_inference_node");
    ros::NodeHandle nh("~");
    LaneDetectionInferenceNode lane_detection_inference_node{nh};
    ros::spin();
    return 0;
}
