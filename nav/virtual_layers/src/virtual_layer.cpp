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
  listener_map.waitForTransform("/map", "/odom", ros::Time(0), ros::Duration(60.0));
}

geometry_msgs::Point VirtualLayer::transform_from_camera_to_odom(double x, double y, double z) {
  geometry_msgs::PoseStamped new_pose = geometry_msgs::PoseStamped();
  new_pose.header.frame_id = "left_camera_link_optical";
  new_pose.pose.position.x = x;
  new_pose.pose.position.y = y;
  new_pose.pose.position.z = z;
  new_pose.pose.orientation.w = 1.0;
  listener.transformPose("/odom", new_pose, new_pose);

  geometry_msgs::Point new_point = geometry_msgs::Point();
  new_point.x = new_pose.pose.position.x;
  new_point.y = new_pose.pose.position.y;
  return new_point;
}

geometry_msgs::Point VirtualLayer::transform_from_odom_to_map(double x, double y) {
  geometry_msgs::PoseStamped new_pose = geometry_msgs::PoseStamped();
  new_pose.header.frame_id = "odom";
  new_pose.pose.position.x = x;
  new_pose.pose.position.y = y;
  new_pose.pose.position.z = 0;
  new_pose.pose.orientation.w = 1.0;
  listener_map.transformPose("/map", new_pose, new_pose);

  geometry_msgs::Point new_point = geometry_msgs::Point();
  new_point.x = new_pose.pose.position.x;
  new_point.y = new_pose.pose.position.y;
  return new_point;
}

void VirtualLayer::clbk(const cv::FloatArray::ConstPtr& msg) {
  cv_points.clear();
  for (int i=0; i < msg->lists.size(); i++) {
    std::vector<geometry_msgs::Point> lane;
    for (int j=0; j< msg->lists[i].elements.size(); j++) {
      double x = msg->lists[i].elements[j].x;
      double y = msg->lists[i].elements[j].y;
      double z = msg->lists[i].elements[j].z;
      lane.push_back(transform_from_camera_to_odom(x, y, z));
    }
    cv_points.push_back(lane);
  }
  geometry_msgs::Point min_pose = transform_from_camera_to_odom(-0.37, -0.657, 3.0420);
  geometry_msgs::Point max_pose = transform_from_camera_to_odom(0.366, 0.655, 1.3089);
  unsigned int min_x, min_y, max_x, max_y;
  worldToMap(min_pose.x + COSTMAP_OFFSET_X, min_pose.y + COSTMAP_OFFSET_Y, min_x, min_y);
  worldToMap(max_pose.x + COSTMAP_OFFSET_X, max_pose.y + COSTMAP_OFFSET_Y, max_x, max_y);
  //worldToMap(min_pose.x, min_pose.y, min_x, min_y);
  //worldToMap(max_pose.x, max_pose.y, max_x, max_y);
  last_min_x = std::min(min_x, max_x);
  last_min_y = std::min(min_y, max_y);
  last_max_x = std::max(min_x, max_x);
  last_max_y = std::max(min_y, max_y);
  //std::cout << last_min_x << " " << last_max_x << " " << last_min_y << " " << last_max_y << std::endl;
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
    //std::cout << x1 << " " << x2 << " " << y1 << " " << y2 << " " << alpha << " " << dis << std::endl;
    for (double k = alpha; k <= 1; k += alpha){
      double x_point = x1 + k * (x2 - x1);
      double y_point = y1 + k * (y2 - y1);
      coord.push_back({x_point, y_point});
      //std::cout << "INSERT" << std::endl;
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
    lane_points.push_back(pointsToLine(cv_points[i], 0.1));
  }
  /*
  for (unsigned int i=last_min_x; i<last_max_x; i++) {
    for (unsigned int j=last_min_y; j<last_min_y; j++) {
      map[i][j] = map[i][j]/2.0;
    }
  }
  */
  for (int i = 0; i < lane_points.size(); i++) {
    for (int j = 0; j < lane_points[i].size(); j++) {
      double magnitude = sqrt(pow(lane_points[i][j][0],2) + pow(lane_points[i][j][1],2));
      // To account for rotation (depends on what frame CV data is in)
      // double mark_x = magnitude*cos(robot_yaw) + robot_x, 
      // mark_y = magnitude*sin(robot_yaw) + robot_y;
      double mark_x = lane_points[i][j][0] - robot_x;
      double mark_y = lane_points[i][j][1] - robot_y;
      geometry_msgs::Point map_xy_point = transform_from_odom_to_map(mark_x, mark_y);
      //std::cout << robot_x << " " << robot_y << " " << mark_x << " " << mark_y << std::endl;
      unsigned int mx;
      unsigned int my;
      if(worldToMap(-(map_xy_point.x) + COSTMAP_OFFSET_X, -(map_xy_point.y) + COSTMAP_OFFSET_Y, mx, my)){
      //if(worldToMap(mark_x, mark_y, mx, my)){
        float x_round = static_cast<float>(static_cast<int>(map_xy_point.x * 10.)) / 10.;
        float y_round = static_cast<float>(static_cast<int>(map_xy_point.y * 10.)) / 10.;
        if (xy_dict.find({x_round, y_round}) == xy_dict.end() ) {
          xy_dict.insert({{x_round, y_round}, 0.5});
        } else {
          xy_dict[{x_round, y_round}] = xy_dict[{x_round, y_round}] + ((1-xy_dict[{x_round, y_round}])*0.5);
        }
      }
      //std::cout << "INSERT " <<  mx << " " << my << " " << mark_x << " " << mark_y << std::endl;
    }
  }

  for (std::map<std::tuple<float,float>, double>::iterator it = xy_dict.begin(); it != xy_dict.end(); it++) {
    double x = static_cast<double>(std::get<0>(it->first));
    double y = static_cast<double>(std::get<1>(it->first));
    unsigned int mx;
    unsigned int my;
    worldToMap(x + COSTMAP_OFFSET_X, y + COSTMAP_OFFSET_Y, mx, my);
    if (it->second > threshold) {
      //std::cout << x << " " << y << std::endl;
      setCost(mx, my, LETHAL_OBSTACLE);
    }
    *min_x = std::min(*min_x, x);
    *min_y = std::min(*min_y, y);
    *max_x = std::max(*max_x, x);
    *max_y = std::max(*max_y, y);
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
