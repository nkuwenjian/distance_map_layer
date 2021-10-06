#include <pluginlib/class_list_macros.h>
#include <chrono>
#include "distance_map_layer/distance_map_layer.h"

PLUGINLIB_EXPORT_CLASS(costmap_2d::DistanceMapLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace costmap_2d
{
DistanceMapLayer::DistanceMapLayer() : distmap_(NULL), binary_map_(NULL), last_size_x_(0), last_size_y_(0)
{
}

DistanceMapLayer::~DistanceMapLayer()
{
  if (distmap_)
    delete[] distmap_;
  if (binary_map_)
    delete[] binary_map_;
}

const double* DistanceMapLayer::getDistanceMap()
{
  boost::mutex::scoped_lock lock(mutex_);
  return distmap_;
}

void DistanceMapLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
      boost::bind(&DistanceMapLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void DistanceMapLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
}

void DistanceMapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // ROS_INFO("min_x = %lf, min_y = %lf, max_x = %lf, max_y = %lf", *min_x, *min_y, *max_x, *max_y);
}

void DistanceMapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  int size_x = master_grid.getSizeInCellsX();
  int size_y = master_grid.getSizeInCellsY();
  resolution_ = master_grid.getResolution();
  const unsigned char* charmap = master_grid.getCharMap();
  // ROS_INFO("size_x = %d, size_y = %d", size_x, size_y);

  if (last_size_x_ != size_x || last_size_y_ != size_y)
  {
    last_size_x_ = size_x;
    last_size_y_ = size_y;
    if (binary_map_)
      delete[] binary_map_;
    if (distmap_)
      delete[] distmap_;

    binary_map_ = new unsigned char[size_x * size_y];
    distmap_ = new double[size_x * size_y];
  }

  for (int i = 0; i < size_x * size_y; i++)
  {
    if (charmap[i] == LETHAL_OBSTACLE)
      binary_map_[i] = 1;
    else
      binary_map_[i] = 0;
  }

  computeCostmap();
}

void DistanceMapLayer::computeCostmap()
{
  boost::mutex::scoped_lock lock(mutex_);

  const auto start_t = std::chrono::system_clock::now();
  cv::Mat gridMapImage(last_size_y_, last_size_x_, CV_8UC1);

  uchar* uchar_ptr = gridMapImage.ptr<uchar>(0);
  for (int i = 0; i < gridMapImage.rows * gridMapImage.cols; ++i)
  {
    if (binary_map_[i] == 1)
    {
      uchar_ptr[i] = 0;
    }
    else if (binary_map_[i] == 0)
    {
      uchar_ptr[i] = 255;
    }
    else
    {
      throw std::runtime_error("The value of occupancy map should be 0 or 1.");
    }
  }

  // calculate the educlidean distance transform via OpenCV distanceTransform function
  cv::Mat distanceFieldImage;
  cv::distanceTransform(gridMapImage, distanceFieldImage, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  float* float_ptr = distanceFieldImage.ptr<float>(0);
  for (int i = 0; i < distanceFieldImage.rows * distanceFieldImage.cols; ++i)
  {
    distmap_[i] = static_cast<double>(float_ptr[i]) * resolution_;
  }

  const auto end_t = std::chrono::system_clock::now();
  std::chrono::duration<double> timediff = end_t - start_t;
  ROS_DEBUG("Runtime = %f ms.", timediff.count() * 1e3);
}

}  // namespace costmap_2d