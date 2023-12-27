//  Copyright (c) 2023, Kirill Ivanov
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#pragma once

#include <vector>

#include "prime_slam/sensor/rgbd.h"

namespace prime_slam {

/**
 * @brief Virtual dataset interface
 */
class IRGBDDataset {
 public:
  /**
   * @brief Returns the RGBD Image at the corresponding index
   * @return RGBD Image
   */
  virtual sensor::RGBDImage operator[](size_t) const = 0;

  /**
   * @brief Returns size of the dataset
   * @return Dataset size
   */
  virtual size_t Size() const noexcept = 0;

  /**
   * @brief Returns groundtruth poses for each frame (RGBD Image)
   * @return Groundtruth poses
   */
  virtual const std::vector<Eigen::Projective3d>& GetGroundTruthPoses()
      const& noexcept = 0;

  /**
   * @brief Returns corresponding camera intrinsics
   * @return Camera intrinsics
   */
  virtual const Eigen::Projective3d& GetIntrinsics() const& noexcept = 0;

  /**
   * @brief Virtual destructor
   */
  virtual ~IRGBDDataset() {}
};

}  // namespace prime_slam
