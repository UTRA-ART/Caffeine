#include <virtual_layers/virtual_layer.h>
#include <pluginlib/class_list_macros.h>

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

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  //random points
  int n = 2;
  int point_x;
  int point_y;
  std::vector<std::vector<int>> points;
  
  for (int xy = 0; xy < n ; xy++){
    point_x = rand() % 639;
    point_y = rand() % 479;
    points.push_back({point_x, point_y});
    cout << point_x << ", " << point_y << endl
  }

  std::vector<std::vector<int>> coord;

  //brensenham's algorithm
  for (int j = 0; j != n; j++){
    int x1 = points[j][0];
    int x2 = points[j+1][0];
    int y1 = points[j][1];
    int y2 = points[j+1][1];
    int h = abs(y2 - y1);
    int w = abs(x2 - x1);
    int dy = (y2 > y1) ? 1 : -1;
    int dx = (x2 > x1) ? 1 : -1;

    if (w > h){
      int y_point = y1;
      int D1 = (2 * h) - w;
      for (int x_point = x1; x_point != x2; x_point += dx){
        if(D1 < 0){
          D1 += 2 * h;
        }
        else{
          y_point += dy;
          D1 += (2 * h) - (2 * w);
          coord.push_back({x_point, y_point});
        }
      }
    }
    else{
      int x_point = x1;
      int D2 = (2 * w) - h;
      for (int y_point = y1; y_point != y2; y_point += dy){
        if (D2 < 0){
          D2 += 2 * w;
        }
        else{
          x_point += dx;
          D2 += (2 * w) - (2 * h);
          coord.push_back({x_point, y_point});
        }
      }
    }
  }

  for (int i = 0; i < coord.size(); i++) {
      double magnitude = sqrt(pow(coord[i][0],2) + pow(coord[i][1],2));
      double mark_x = magnitude*cos(robot_yaw) + robot_x, mark_y = magnitude*sin(robot_yaw) + robot_y;
      // double mark_x = coord[i][0] + robot_x, mark_y = coord[i][1] + robot_y;
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
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

}  // namespace virtual_layers
