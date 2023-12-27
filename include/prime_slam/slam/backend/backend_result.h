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

#include "prime_slam/slam/backend/optimized_landmark_position.h"
#include "prime_slam/slam/backend/optimized_pose.h"

namespace prime_slam::backend {

/**
 * @brief Result of the backend: optimized poses and landmarks
 */
struct BackendResult {
  std::vector<OptimizedLandmarkPosition> optimized_positions;
  std::vector<OptimizedPose> optimized_poses;
};

}  // namespace prime_slam::backend
