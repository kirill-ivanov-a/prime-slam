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
#include <utility>

#include "prime_slam/slam/frame/frame.h"

namespace prime_slam::factor_graph {

/**
 * @brief Class representing frame pose in a factor graph
 */
class PoseNode final {
 public:
  /**
   * @brief Default constructor
   */
  PoseNode() = default;

  /**
   * @brief Creates pose node from given ID and pose
   * @param id ID of corresponding frame
   * @param world_to_camera_transform
   */
  PoseNode(size_t id, Eigen::Projective3d world_to_camera_transform)
      : id_(id),
        world_to_camera_transform_(std::move(world_to_camera_transform)) {}

  /**
   * @brief Returns current pose
   * @return Current pose
   */
  [[nodiscard]] const Eigen::Projective3d& GetPose() const& noexcept {
    return world_to_camera_transform_;
  }

  /**
   * @brief Sets new pose
   * @param new_pose New pose
   */
  void SetPose(Eigen::Projective3d new_pose) {
    world_to_camera_transform_ = std::move(new_pose);
  }

  /**
   * @brief Returns ID of the pose node
   * @return ID
   */
  [[nodiscard]] size_t GetID() const noexcept { return id_; }

 private:
  size_t id_{0};
  Eigen::Projective3d world_to_camera_transform_;
};

}  // namespace prime_slam::factor_graph
