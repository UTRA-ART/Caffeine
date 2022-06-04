#ifndef VIRTUAL_LAYER_H_
#define VIRTUAL_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv/FloatArray.h>
#include <tf/transform_listener.h>

namespace virtual_layers
{

class VirtualLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  VirtualLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }
  
  virtual void matchSize();

  std::vector<std::vector<geometry_msgs::Point>> cv_points;
  void clbk(const cv::FloatArray::ConstPtr& msg);
  geometry_msgs::Point transform_from_camera_to_map(double x, double y, double z);
  geometry_msgs::Point transform_from_map_to_baselink(double x, double y, double z=0);
  
  tf::TransformListener listener_map;
  tf::TransformListener listener_baselink;

private:
  const double COSTMAP_OFFSET_X = 50.0; //0.5 * global map (length/width)
  const double COSTMAP_OFFSET_Y = 50.0; //0.5 * global map (length/width)
  std::map<std::tuple<unsigned int,unsigned int>, std::tuple<double, double>> xy_dict_in_map_frame;
  double map[1000][1000] = {0};
  double threshold = 0.97; // threshold to declare lethal obstacle in costmap

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  unsigned int last_min_x;
  unsigned int last_max_x;
  unsigned int last_min_y;
  unsigned int last_max_y;


  ros::NodeHandle nh;
  ros::Subscriber cv_sub;
};
}
#endif //  VIRTUAL_LAYER_H_