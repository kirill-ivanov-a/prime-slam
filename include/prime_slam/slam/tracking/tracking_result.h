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

#include <Eigen/Geometry>

#include "prime_slam/slam/data_association.h"

namespace prime_slam::tracking {

/**
 * @brief Result of tracker
 */
struct TrackingResult {
  /**
   * @brief Change of the pose of the new frame with respect to the map or to
   * another frame
   */
  Eigen::Projective3d pose;
  /**
   * @brief Associations between observations of the new frame and the map's
   * landmarks or observations of the previous frame
   */
  DataAssociation associations;
};

}  // namespace prime_slam::tracking
