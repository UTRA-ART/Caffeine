#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <vector> 

namespace simple_layer_namespace
{

class SimpleLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  SimpleLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  double counter = 0.0;

private:
  const double COSTMAP_OFFSET_X = 3.0;
  const double COSTMAP_OFFSET_Y = 3.0;
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif

