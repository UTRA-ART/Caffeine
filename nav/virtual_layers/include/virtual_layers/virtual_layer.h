#ifndef VIRTUAL_LAYER_H_
#define VIRTUAL_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <cv_pkg/cv_msg.h>
#include <vector> 


namespace virtual_layers
{

class VirtualLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  VirtualLayer();
  void clbk(const cv_pkg::cv_msg::ConstPtr& msg);

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }
  bool initialize = false;
  std::vector<std::vector<double>> points;
  
  virtual void matchSize();

  double counter = 0.0;

  //void resetLastUpdated();

private:
  const double COSTMAP_OFFSET_X = 25.0; //4.0;
  const double COSTMAP_OFFSET_Y = 25.0; //4.0;
  
  ros::NodeHandle nh;
  ros::Subscriber cv_sub;

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif //  VIRTUAL_LAYER_H_

