#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <cv_pkg/cv_msg.h>

#include <vector>

std::vector<std::vector<double>> coords;
using namespace std;

void clbk(const cv_pkg::cv_msg::ConstPtr& msg) {
  cout << "HI" << msg << endl;
  ROS_INFO("%d", msg->another_field);
  ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
  coords.push_back({msg->points[0].x, msg->points[0].y});
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "cv_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("update", 1, clbk);

  ros::spin();

}