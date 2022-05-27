#ifndef VIRTUAL_LAYER_H_
#define VIRTUAL_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <cv_pkg/cv_msg.h>
#include <geometry_msgs/Point.h>

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

  std::vector<geometry_msgs::Point> cv_points;
  void clbk(const cv_pkg::cv_msg::ConstPtr& msg);
  double counter = 0.0;

private:
  const double COSTMAP_OFFSET_X = 4.0; //4.0;
  const double COSTMAP_OFFSET_Y = 4.0; //4.0;

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  ros::NodeHandle nh;
  ros::Subscriber cv_sub;
};
}
#endif //  VIRTUAL_LAYER_H_
