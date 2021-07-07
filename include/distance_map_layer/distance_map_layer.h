#ifndef DISTANCE_MAP_LAYER_H_
#define DISTANCE_MAP_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

namespace distance_map_layer
{
class DistanceMapLayer : public costmap_2d::Layer
{
public:
  DistanceMapLayer();
  virtual ~DistanceMapLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual const double* getDistanceMap();

private:
  void computeCostmap();
  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;

  double* distmap_;
  unsigned char* binary_map_;
  int last_size_x_;
  int last_size_y_;
  double resolution_;
  boost::mutex mutex_;
};

}  // namespace distance_map_layer

#endif  // DISTANCE_MAP_LAYER_H_