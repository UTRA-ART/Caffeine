// #include "ros/ros.h"
// #include <geometry_msgs/PoseArray.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>

// class RampNavigateNode {
// public:
//     RampNavigateNode() : ac("/move_base", true) {}
//     void begin() {

//         ramp_front_top = nh.subscribe("ramp_seg", 10, &RampNavigateNode::rampFrontCallback, this);
//     }

// private:
//     actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

//     void rampFrontCallback(const geometry_msgs::PoseArrayConstPtr& lidar_msg) {

//     }
// };

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "ramp_navigate");
//     ros::NodeHandle nh("~");

//     RampNavigateNode node;
//     node.begin(nh);

//     ros::spin();
// }

int main() {}