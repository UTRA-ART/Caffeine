#include <virtual_layers/virtual_layer.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <random>

using namespace std;

PLUGINLIB_EXPORT_CLASS(virtual_layers::VirtualLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace virtual_layers
{

VirtualLayer::VirtualLayer() {}

void VirtualLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

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

std::vector<std::vector<double>> getRandomPoints() {
  int n = 3;
  double point_x;
  double point_y;
  std::vector<std::vector<double>> points;
  std::random_device rd_x;
  std::random_device rd_y;
  std::mt19937 gen_x(rd_x());
  std::mt19937 gen_y(rd_y());
  std::uniform_real_distribution<double> rand_x(-4, 4);
  std::uniform_real_distribution<double> rand_y(-4, 4);

  for (int xy = 0; xy < n ; xy++){
    point_x = rand_x(rd_x);
    point_y = rand_y(rd_y);
    points.push_back({point_x, point_y});
    ROS_INFO("%f %f" , point_x, point_y);
  }

  return points;
}

std::vector<std::vector<double>> pointsToLine(std::vector<std::vector<double>> points, double t) {
  std::vector<std::vector<double>> coord;
  coord.push_back({points[0][0], points[0][1]});
  int m = points.size() - 1;
  for (int j = 0; j < m; j++) {
    double x1 = points[j][0];
    double x2 = points[j+1][0];
    double y1 = points[j][1];
    double y2 = points[j+1][1];
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

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  
  if (initialize != true) {
    initialize = true;
    points = getRandomPoints();
  }

  cout << "------------------" << endl;
  cout << "robot_x:   " << robot_x << endl;
  cout << "robot_y:   " << robot_y << endl;
  cout << "robot_yaw: " << robot_yaw << endl;
  cout << "min_x:     " << *min_x << endl;
  cout << "min_y:     " << *min_y << endl;
  cout << "max_x:     " << *max_x << endl;
  cout << "max_y:     " << *max_y << endl;

  //points = {{-8, 8}, {0, 5}};
  //std::vector<std::vector<double>> coord = pointsToLine(points, 0.05);
  std::vector<std::vector<double>> coord = {{2, 2}};

  for (int i = 0; i < coord.size(); i++) {
      double magnitude = sqrt(pow(coord[i][0],2) + pow(coord[i][1],2));
      //double mark_x = magnitude*cos(robot_yaw) + robot_x, 
            // mark_y = magnitude*sin(robot_yaw) + robot_y;
      double mark_x = coord[i][0] + robot_x, mark_y = coord[i][1] + robot_y;
      cout << "coord_x:   " << coord[i][0] << endl;
      cout << "coord_y:   " << coord[i][1] << endl;
      cout << "mark_x:    " << mark_x << endl;
      cout << "mark_y:    " << mark_y << endl;
      cout << "COSTMAP_OFFSET_X: " << COSTMAP_OFFSET_X << endl;
      cout << "COSTMAP_OFFSET_X: " << COSTMAP_OFFSET_X << endl;
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
      {
        continue;
      }
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

}  // namespace virtual_layers
