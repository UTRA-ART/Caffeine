#include <virtual_layers/virtual_layer.h>
#include <pluginlib/class_list_macros.h>
#include <random>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(virtual_layers::VirtualLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace virtual_layers
{
VirtualLayer::VirtualLayer() {
  listener.waitForTransform("/odom", "/left_camera_link_optical", ros::Time(0), ros::Duration(60.0));
}

void VirtualLayer::clbk(const cv::FloatArray::ConstPtr& msg) {
  cv_points.clear();
  for (int i=0; i < msg->lists.size(); i++) {
    std::vector<geometry_msgs::Point> lane;
    for (int j=0; j< msg->lists[i].elements.size(); j++) {
      geometry_msgs::PoseStamped new_pose = geometry_msgs::PoseStamped();
      new_pose.header.frame_id = "left_camera_link_optical";
      new_pose.pose.position.x = msg->lists[i].elements[j].x;
      new_pose.pose.position.y = msg->lists[i].elements[j].y;
      new_pose.pose.position.z = msg->lists[i].elements[j].z;
      new_pose.pose.orientation.w = 1.0;
      listener.transformPose("/odom", new_pose, new_pose);

      geometry_msgs::Point new_point = geometry_msgs::Point();
      new_point.x = new_pose.pose.position.x;
      new_point.y = new_pose.pose.position.y;
      //std::cout << new_point.x << " " << new_point.y << std::endl;
      lane.push_back(new_point);
    }
    cv_points.push_back(lane);
  }
}//callback function

void VirtualLayer::onInitialize()
{
  nh = ros::NodeHandle("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();
  
  //subscribe
  cv_sub=nh.subscribe("/cv/lane_detections", 10, &VirtualLayer::clbk, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &VirtualLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void VirtualLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void VirtualLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

std::vector<std::vector<double>> pointsToLine(std::vector<geometry_msgs::Point> points, double t) {
  std::vector<std::vector<double>> coord;
  if (points.empty())
  {
    return coord;
  }
  
  coord.push_back({points[0].x, points[0].y});
  for (int j = 0; j < points.size()-1; j++) {
    double x1 = points[j].x;
    double x2 = points[j+1].x;
    double y1 = points[j].y;
    double y2 = points[j+1].y;
    double dis = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    double alpha = t / dis;
    for (double k = alpha; k <= 1; k += alpha){
      double x_point = x1 + k * (x2 - x1);
      double y_point = y1 + k * (y2 - y1);
      coord.push_back({x_point, y_point});
    }
  } 
  
  return coord;
}

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  std::vector<std::vector<std::vector<double>>> lane_points;

  for (int i = 0; i < cv_points.size(); i++) {
    lane_points.push_back(pointsToLine(cv_points[i], 0.05));
  }

  for (int i = 0; i < lane_points.size(); i++) {
    for (int j = 0; j < lane_points[i].size(); j++) {
      double magnitude = sqrt(pow(lane_points[i][j][0],2) + pow(lane_points[i][j][1],2));
      // To account for rotation (depends on what frame CV data is in)
      // double mark_x = magnitude*cos(robot_yaw) + robot_x, 
      // mark_y = magnitude*sin(robot_yaw) + robot_y;
      double mark_x = lane_points[i][j][0] - robot_x;
      double mark_y = lane_points[i][j][1] - robot_y;
      unsigned int mx;
      unsigned int my;
      if(worldToMap(mark_x + COSTMAP_OFFSET_X, mark_y + COSTMAP_OFFSET_Y, mx, my)){
        setCost(mx, my, LETHAL_OBSTACLE);
      }
      
      *min_x = std::min(*min_x, mark_x);
      *min_y = std::min(*min_y, mark_y);
      *max_x = std::max(*max_x, mark_x);
      *max_y = std::max(*max_y, mark_y);
    }
  }
}

void VirtualLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}
}  // namespace virtual_layers
