#include <virtual_layers/virtual_layer_global.h>
#include <pluginlib/class_list_macros.h>
#include <random>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(virtual_layers_global::VirtualLayer_Global, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace virtual_layers_global
{
VirtualLayer_Global::VirtualLayer_Global() {
  listener.waitForTransform(target_frame, camera_frame, ros::Time(0), ros::Duration(60.0));
}

geometry_msgs::Point VirtualLayer_Global::transform_from_camera_to_odom(double x, double y, double z) {
  geometry_msgs::PoseStamped new_pose = geometry_msgs::PoseStamped();
  new_pose.header.frame_id = camera_frame;
  new_pose.pose.position.x = x;
  new_pose.pose.position.y = y;
  new_pose.pose.position.z = z;
  new_pose.pose.orientation.w = 1.0;
  listener.transformPose(target_frame, new_pose, new_pose);

  geometry_msgs::Point new_point = geometry_msgs::Point();
  new_point.x = new_pose.pose.position.x;
  new_point.y = new_pose.pose.position.y;
  return new_point;
}

geometry_msgs::Point VirtualLayer_Global::transform_from_baselink_to_odom(double x, double y, double z) {
  geometry_msgs::PoseStamped new_pose = geometry_msgs::PoseStamped();
  new_pose.header.frame_id = "/base_link";
  new_pose.pose.position.x = x;
  new_pose.pose.position.y = y;
  new_pose.pose.position.z = z;
  new_pose.pose.orientation.w = 1.0;
  listener.transformPose(target_frame, new_pose, new_pose);

  geometry_msgs::Point new_point = geometry_msgs::Point();
  new_point.x = new_pose.pose.position.x;
  new_point.y = new_pose.pose.position.y;
  return new_point;
}

void VirtualLayer_Global::clbk(const cv::FloatArray::ConstPtr& msg) {
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
  
}//callback function

void VirtualLayer_Global::onInitialize()
{
  nh = ros::NodeHandle("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();
  
  //subscribe
  cv_sub=nh.subscribe("/cv/lane_detections", 10, &VirtualLayer_Global::clbk, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &VirtualLayer_Global::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void VirtualLayer_Global::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void VirtualLayer_Global::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
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

void VirtualLayer_Global::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  std::vector<std::vector<std::vector<double>>> lane_points;

  for (int i = 0; i < cv_points.size(); i++) {
    lane_points.push_back(pointsToLine(cv_points[i], 0.1));
  }
  // Get trapezoid (camera's field of view in odom frame) to decay;
  double min_x_in_baselink = 1;
  double max_x_in_baselink = 3.35 + min_x_in_baselink;
  double min_y_in_baselink = -2.375;
  double max_y_in_baselink = 2.375;

  for (double i=min_x_in_baselink; i<max_x_in_baselink; i += 0.1) {
    for (double j=min_y_in_baselink; j<max_y_in_baselink; j += 0.1) {
      geometry_msgs::Point odom_point = transform_from_baselink_to_odom(i, j, 0);
      unsigned int odom_grid_x, odom_grid_y;
      worldToMap(odom_point.x + COSTMAP_OFFSET_X, odom_point.y + COSTMAP_OFFSET_Y, odom_grid_x, odom_grid_y);
      map[odom_grid_x][odom_grid_y] = map[odom_grid_x][odom_grid_y]/2.0;
    
      *min_x = std::min(*min_x, odom_point.x);
      *min_y = std::min(*min_y, odom_point.y);
      *max_x = std::max(*max_x, odom_point.x);
      *max_y = std::max(*max_y, odom_point.y);
    }
  }

  for (int i = 0; i < lane_points.size(); i++) {
    for (int j = 0; j < lane_points[i].size(); j++) {
      double map_world_x = lane_points[i][j][0];
      double map_world_y = lane_points[i][j][1];

      unsigned int map_grid_x, map_grid_y;
      worldToMap(map_world_x+COSTMAP_OFFSET_X, map_world_y+COSTMAP_OFFSET_Y, map_grid_x, map_grid_y);
      map[map_grid_x][map_grid_y] = map[map_grid_x][map_grid_y] + ((1-map[map_grid_x][map_grid_y])*0.5);
      map[map_grid_x][map_grid_y] = map[map_grid_x][map_grid_y] + ((1-map[map_grid_x][map_grid_y])*0.5);

      if (xy_dict_in_map_frame.find({map_grid_x, map_grid_y}) == xy_dict_in_map_frame.end()) {
        xy_dict_in_map_frame.insert({{map_grid_x, map_grid_y}, {map_world_x, map_world_y}});
      }
    }
  }

  for (std::map<std::tuple<unsigned int,unsigned int>, std::tuple<double,double>>::iterator it = xy_dict_in_map_frame.begin(); it != xy_dict_in_map_frame.end(); it++) {
    unsigned int map_grid_x = std::get<0>(it->first);
    unsigned int map_grid_y = std::get<1>(it->first);

    double costmap_world_x = std::get<0>(it->second) - robot_x;
    double costmap_world_y = std::get<1>(it->second) - robot_y;
    unsigned int grid_x = static_cast<unsigned int>(10*(costmap_world_x + COSTMAP_OFFSET_X));
    unsigned int grid_y = static_cast<unsigned int>(10*(costmap_world_y + COSTMAP_OFFSET_Y));
    
    if (map[map_grid_x][map_grid_y] > threshold) {
      setCost(grid_x, grid_y, LETHAL_OBSTACLE);
    }
    *min_x = std::min(*min_x, costmap_world_x);
    *min_y = std::min(*min_y, costmap_world_y);
    *max_x = std::max(*max_x, costmap_world_x);
    *max_y = std::max(*max_y, costmap_world_y);
  }
}

void VirtualLayer_Global::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
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

  resetMap(min_i, min_j, max_i, max_j);
}
}  // namespace virtual_layers_global