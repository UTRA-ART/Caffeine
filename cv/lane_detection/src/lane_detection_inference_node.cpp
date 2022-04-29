#include <boost/bind/placeholders.hpp>

#include "lane_detection/lane_detection_inference_node.h"

#include "cv_bridge/cv_bridge.h"

LaneDetectionInferenceNode::LaneDetectionInferenceNode(ros::NodeHandle nh)
    : nh_{nh}, it_{nh_},
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

    cv::Mat output_image;

    lane_detection_mask_pub_.publish(
        cv_bridge::CvImage(image_msg->header,
                           sensor_msgs::image_encodings::MONO8, output_image)
            .toImageMsg());
}

int main(int argc, char **argv) { return 0; }
