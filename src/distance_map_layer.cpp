/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "distance_map_layer/distance_map_layer.h"

#include <chrono>  // NOLINT

#include "glog/logging.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(costmap_2d::DistanceMapLayer, costmap_2d::Layer)

namespace costmap_2d {

void DistanceMapLayer::onInitialize() {
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = std::make_unique<
      dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&DistanceMapLayer::ReconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void DistanceMapLayer::ReconfigureCB(
    const costmap_2d::GenericPluginConfig& config, uint32_t level) {
  enabled_ = config.enabled;
}

void DistanceMapLayer::updateBounds(double robot_x, double robot_y,
                                    double robot_yaw, double* min_x,
                                    double* min_y, double* max_x,
                                    double* max_y) {}

void DistanceMapLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                   int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_) {
    LOG(ERROR) << "DistanceMapLayer is disable.";
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  double resolution = master_grid.getResolution();
  VLOG(4) << std::fixed << "size_x: " << size_x << ", size_y: " << size_y
          << ", resolution: " << resolution;

  ReallocateMemory(size_x, size_y);

  if (!UpdateBinaryMap(master_grid, size_x, size_y)) {
    LOG(ERROR) << "Failed to update binary map.";
    return;
  }

  ComputeDistanceMap(size_x, size_y, resolution);
}

void DistanceMapLayer::ReallocateMemory(unsigned int size_x,
                                        unsigned int size_y) {
  if (last_size_x_ == size_x && last_size_y_ == size_y) {
    return;
  }

  binary_map_.clear();
  binary_map_.resize(size_x * size_y);
  euclidean_distance_map_.clear();
  euclidean_distance_map_.resize(size_x * size_y);
  last_size_x_ = size_x;
  last_size_y_ = size_y;
}

bool DistanceMapLayer::UpdateBinaryMap(const costmap_2d::Costmap2D& master_grid,
                                       unsigned int size_x,
                                       unsigned int size_y) {
  const uint8_t* char_map = master_grid.getCharMap();
  if (char_map == nullptr) {
    LOG(ERROR) << "char_map == nullptr";
    return false;
  }

  for (unsigned int i = 0U; i < size_x * size_y; ++i) {
    if (char_map[i] == LETHAL_OBSTACLE) {
      binary_map_[i] = 1U;
    } else {
      binary_map_[i] = 0U;
    }
  }
  return true;
}

void DistanceMapLayer::ComputeDistanceMap(unsigned int size_x,
                                          unsigned int size_y,
                                          double resolution) {
  const auto start_timestamp = std::chrono::system_clock::now();
  cv::Mat grid_map_image(size_y, size_x, CV_8UC1);

  uchar* uchar_ptr = grid_map_image.ptr<uchar>(0);
  for (unsigned int i = 0U; i < size_x * size_y; ++i) {
    if (binary_map_[i] == 1U) {
      uchar_ptr[i] = 0U;
    } else {
      uchar_ptr[i] = 255U;
    }
  }

  // Calculate educlidean distances via OpenCV distanceTransform.
  cv::Mat distance_field_image;
  cv::distanceTransform(grid_map_image, distance_field_image, cv::DIST_L2,
                        cv::DIST_MASK_PRECISE);

  float* float_ptr = distance_field_image.ptr<float>(0);
  for (unsigned int i = 0U; i < size_x * size_y; ++i) {
    euclidean_distance_map_[i] = static_cast<double>(float_ptr[i]) * resolution;
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> timediff = end_timestamp - start_timestamp;
  VLOG(4) << "Runtime: " << timediff.count() * 1e3 << " ms.";
}

}  // namespace costmap_2d
