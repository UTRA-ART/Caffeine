#include <virtual_layers/virtual_layer.h>
#include <pluginlib/class_list_macros.h>
#include <random>
#include <iostream>
#include <math.h>       /* atan */

PLUGINLIB_EXPORT_CLASS(virtual_layers::VirtualLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace virtual_layers
{
VirtualLayer::VirtualLayer() {
  listener_map.waitForTransform("/map", "/left_camera_link_optical", ros::Time(0), ros::Duration(60.0));
  listener_baselink.waitForTransform("/odom", "/map", ros::Time(0), ros::Duration(60.0));
}

geometry_msgs::Point VirtualLayer::transform_from_camera_to_map(double x, double y, double z) {
  geometry_msgs::PoseStamped new_pose = geometry_msgs::PoseStamped();
  new_pose.header.frame_id = "left_camera_link_optical";
  new_pose.pose.position.x = x;
  new_pose.pose.position.y = y;
  new_pose.pose.position.z = z;
  new_pose.pose.orientation.w = 1.0;
  listener_map.transformPose("/map", new_pose, new_pose);

  geometry_msgs::Point new_point = geometry_msgs::Point();
  new_point.x = new_pose.pose.position.x;
  new_point.y = new_pose.pose.position.y;
  return new_point;
}

geometry_msgs::Point VirtualLayer::transform_from_map_to_baselink(double x, double y, double z) {
  geometry_msgs::PoseStamped new_pose = geometry_msgs::PoseStamped();
  new_pose.header.frame_id = "map";
  new_pose.pose.position.x = x;
  new_pose.pose.position.y = y;
  new_pose.pose.position.z = z;
  new_pose.pose.orientation.w = 1.0;
  listener_baselink.transformPose("/odom", new_pose, new_pose);

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
      //unsigned mx, my;
      //worldToMap(x+COSTMAP_OFFSET_X, y+COSTMAP_OFFSET_Y, mx, my);
      //setCost(mx, my, LETHAL_OBSTACLE);
      lane.push_back(transform_from_camera_to_map(x, y, z));
    }
    cv_points.push_back(lane);
  }
  // TODO: Min/Max xy bounds broken prob
  geometry_msgs::Point min_point = transform_from_camera_to_map(-0.37, -0.657, 3.0420);
  geometry_msgs::Point max_point = transform_from_camera_to_map(0.366, 0.655, 1.3089);

  //TODO: min max bounds we dont think work
  unsigned int min_x = static_cast<unsigned int>(10*(min_point.x + COSTMAP_OFFSET_X));
  unsigned int min_y = static_cast<unsigned int>(10*(min_point.y + COSTMAP_OFFSET_Y));
  unsigned int max_x = static_cast<unsigned int>(10*(max_point.x + COSTMAP_OFFSET_X));
  unsigned int max_y = static_cast<unsigned int>(10*(max_point.y + COSTMAP_OFFSET_Y));
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
  
  for (unsigned int i=last_min_x; i<last_max_x; i++) {
    for (unsigned int j=last_min_y; j<last_min_y; j++) {
      map[i][j] = map[i][j]/2.0;
    }
  }

  for (int i = 0; i < lane_points.size(); i++) {
    for (int j = 0; j < lane_points[i].size(); j++) {
      //double magnitude = sqrt(pow(lane_points[i][j][0],2) + pow(lane_points[i][j][1],2));
      // To account for rotation (depends on what frame CV data is in)
      //double mark_x = magnitude*cos(robot_yaw);// + robot_x; 
      //double mark_y = magnitude*sin(robot_yaw);// + robot_y;
      double mark_x = lane_points[i][j][0];
      double mark_y = lane_points[i][j][1];
      unsigned mx, my;
      //worldToMap(mark_x+COSTMAP_OFFSET_X, mark_y+COSTMAP_OFFSET_Y, mx, my);
      //setCost(mx, my, LETHAL_OBSTACLE);
      //std::cout << "baselink: " << mark_x << " " << mark_y <<  " " << atan(mark_y/mark_x) * 180 / 3.14159265 << std::endl;
      //geometry_msgs::PoseStamped map_pose = transform_from_baselink_to_map(mark_x, mark_y, 0, robot_yaw); // map frame xy meters
      //std::cout << "map: " << map_point.x << " " << map_point.y <<  " " << atan(map_point.y/map_point.x) * 180 / 3.14159265 << std::endl;
      //std::cout << "baselink-map: " << (atan(mark_y/mark_x) * 180 / 3.14159265) - (atan(map_point.y/map_point.x) * 180 / 3.14159265) << " | robot: " << robot_yaw  << std::endl;
      //std::cout << robot_x << " " << robot_y << " " << robot_yaw << std::endl;
      // map frame xy grid
      //unsigned int mx, my;
      //worldToMap(map_pose.pose.position.x + COSTMAP_OFFSET_X, map_pose.pose.position.y + COSTMAP_OFFSET_Y, mx, my); 
      mx = static_cast<unsigned int>(10*(mark_x + COSTMAP_OFFSET_X));
      my = static_cast<unsigned int>(10*(mark_y + COSTMAP_OFFSET_Y));
      map[mx][my] = map[mx][my] + ((1-map[mx][my])*0.5);
      map[mx][my] = map[mx][my] + ((1-map[mx][my])*0.5);
      //std::cout << mx << " " << my << " " << map_point.x << " " << map_point.y << std::endl;
      if (xy_dict_in_map_frame.find({mx, my}) == xy_dict_in_map_frame.end()) {
        xy_dict_in_map_frame.insert({{mx, my}, {mark_x, mark_y}});
      }
      //std::cout << "INSERT " <<  mx << " " << my << " " << mark_x << " " << mark_y << std::endl;
    }
  }

  setCost(500, 500, LETHAL_OBSTACLE);
  setCost(500, 510, LETHAL_OBSTACLE);
  for (std::map<std::tuple<unsigned int,unsigned int>, std::tuple<double,double>>::iterator it = xy_dict_in_map_frame.begin(); it != xy_dict_in_map_frame.end(); it++) {
    
    unsigned int map_grid_x = std::get<0>(it->first);
    unsigned int map_grid_y = std::get<1>(it->first);
    double map_world_x = std::get<0>(it->second) - robot_x;
    double map_world_y = std::get<1>(it->second) - robot_y;
    //geometry_msgs::Point baselink_point = transform_from_map_to_baselink(map_world_x, map_world_y, 0);
    //double magnitude = sqrt(pow(baselink_point.x,2) + pow(baselink_point.y,2));
    //double theta = atan(baselink_point.y / baselink_point.x);
    //baselink_point.x = magnitude*cos(-robot_yaw) + robot_x;
    //baselink_point.y = magnitude*sin(-robot_yaw) + robot_y;
    //std::cout << "HI: " << odom_point.x << " " << odom_point.y << " " << odom_grid_x << " " << odom_grid_y << " " << map[max_grid_x][max_grid_y] << " " << max_grid_x << " " << max_grid_y << std::endl;
    if (map[map_grid_x][map_grid_y] > threshold) {
      unsigned int grid_x = static_cast<unsigned int>(10*(map_world_x + COSTMAP_OFFSET_X));
      unsigned int grid_y = static_cast<unsigned int>(10*(map_world_y + COSTMAP_OFFSET_Y));
      setCost(grid_x, grid_y, LETHAL_OBSTACLE);
      //continue;
    }
    
    *min_x = std::min(*min_x, map_world_x);
    *min_y = std::min(*min_y, map_world_y);
    *max_x = std::max(*max_x, map_world_x);
    *max_y = std::max(*max_y, map_world_y);
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
  /*

  for (std::map<std::tuple<unsigned int,unsigned int>, std::tuple<double,double>>::iterator it = xy_dict_in_map_frame.begin(); it != xy_dict_in_map_frame.end(); it++) {
    
    unsigned int map_grid_x = std::get<0>(it->first);
    unsigned int map_grid_y = std::get<1>(it->first);
    double map_world_x = std::get<0>(it->second);
    double map_world_y = std::get<1>(it->second);
    geometry_msgs::Point baselink_pose = transform_from_map_to_baselink(map_world_x, map_world_y, 0);
    //double magnitude = sqrt(pow(baselink_pose.pose.position.x,2) + pow(baselink_pose.pose.position.y,2));
    //double theta = atan(baselink_point.y / baselink_point.x);
    //baselink_point.x = magnitude*cos(map_world_yaw);
    //baselink_point.y = magnitude*sin(map_world_yaw);
    unsigned int baselink_grid_x, baselink_grid_y;
    
    worldToMap(baselink_pose.x + COSTMAP_OFFSET_X, baselink_pose.y + COSTMAP_OFFSET_Y, baselink_grid_x, baselink_grid_y);
    //std::cout << "HI: " << odom_point.x << " " << odom_point.y << " " << odom_grid_x << " " << odom_grid_y << " " << map[max_grid_x][max_grid_y] << " " << max_grid_x << " " << max_grid_y << std::endl;
    if (map_grid_x < max_i && map_grid_x >= min_i && map_grid_y < max_j && map_grid_y >= min_j) {
      if (map[map_grid_x][map_grid_y] > threshold) {
        master_grid.setCost(baselink_grid_x, baselink_grid_y, LETHAL_OBSTACLE);
        continue;
      }
    }
  }
  //master_grid.setCost(500,530,LETHAL_OBSTACLE);
  */
}
}  // namespace virtual_layers