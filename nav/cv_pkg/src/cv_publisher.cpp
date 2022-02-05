#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <cv_pkg/cv_msg.h>

#include <vector>

struct Point {
    double x;
    double y;
};

int main(int argc, char **argv)
{
    // ROS objects
    ros::init(argc, argv, "cv_publisher");
    ros::NodeHandle n;
    ros::Publisher cv_pub = n.advertise<cv_pkg::cv_msg>("update", 1);
    ros::Rate loop_rate(1);

    // define message to publish
    cv_pkg::cv_msg msg;
    msg.another_field = 0;

    // creating vector = {[-1,-1], [-1,1]}
    Point dummy_array[2];
    Point point;
    for (int i = 0; i < 2; i++) {
        point.x = -1;
        point.y = -1 + (2*i);
        dummy_array[i] = point;
    }
    std::vector<Point> dummy_vector (dummy_array, dummy_array + sizeof(dummy_array)/sizeof(Point));
    
    // loop control
    int count = 0;
    while(ros::ok())
    {
        msg.points.clear();
        msg.another_field = count;
        int i =0;
        for (std::vector<Point>::iterator it = dummy_vector.begin(); it != dummy_vector.end(); it++)
        {
            geometry_msgs::Point point;
            point.x = (*it).x;
            point.y = (*it).y;
            point.z = 0;
            msg.points.push_back(point);
            i++;
        }
        
        ROS_INFO("%d", msg.another_field);
        cv_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
