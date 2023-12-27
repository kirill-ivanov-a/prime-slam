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

#include <cstddef>

#include <Eigen/Core>

namespace prime_slam::backend {

/**
 * @brief Optimized Landmark Position
 */
struct OptimizedLandmarkPosition {
  /**
   * @brief landmark id
   */
  size_t id;
  /**
   * @brief optimized position
   */
  Eigen::VectorXd position;
};

}  // namespace prime_slam::backend
