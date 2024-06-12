#include "cv/FloatArray.h"
#include "geometry_msgs/Point.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"

class FloatArrayToPointCloud2Node
{
  public:
    FloatArrayToPointCloud2Node(ros::NodeHandle nh);
    void floatArraycallback(const cv::FloatArray::ConstPtr &msg);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber float_array_sub_;
    ros::Publisher points2_pub_;
};

FloatArrayToPointCloud2Node::FloatArrayToPointCloud2Node(ros::NodeHandle nh)
    : nh_{nh}, float_array_sub_{nh_.subscribe(
                   "/cv/lane_detections", 10,
                   &FloatArrayToPointCloud2Node::floatArraycallback, this)},
      points2_pub_{nh_.advertise<sensor_msgs::PointCloud2>(
          "/cv/lane_detections_cloud", 1)}
{
}

void FloatArrayToPointCloud2Node::floatArraycallback(
    const cv::FloatArray::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> output_cloud{};
    for (const cv::FloatList &list : msg->lists) {
        for (const geometry_msgs::Point &element : list.elements) {
            pcl::PointXYZ point = pcl::PointXYZ(static_cast<float>(element.x),
                                                static_cast<float>(element.y),
                                                static_cast<float>(element.z));
            output_cloud.push_back(point);
        }
    }

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    points2_pub_.publish(output_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "float_array_to_pc2_node");
    ros::NodeHandle nh("~");
    FloatArrayToPointCloud2Node float_array_to_pc2_node(nh);
    ros::spin();
    return 0;
}
