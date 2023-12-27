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

#include "prime_slam/slam/mapping/landmark.h"

namespace prime_slam::factor_graph {

/**
 * @brief Class representing map landmark in a factor graph
 */
class LandmarkNode final {
 public:
  /**
   * @brief Default constructor
   */
  LandmarkNode() = default;

  /**
   * @brief Creates landmark node from a given landmark
   * @param landmark Landmark object
   */
  LandmarkNode(const mapping::Landmark& landmark)
      : id_(landmark.GetID()), position_(landmark.GetPosition()) {}

  /**
   * @brief Creates landmark node from given ID and position
   * @param landmark_id ID of corresponding landmark
   * @param landmark_position Position of corresponding landmark
   */
  LandmarkNode(size_t landmark_id, Eigen::VectorXd landmark_position)
      : id_(landmark_id), position_(std::move(landmark_position)) {}

  /**
   * @brief Returns current position
   * @return Position
   */
  [[nodiscard]] const Eigen::VectorXd& GetPosition() const& noexcept {
    return position_;
  }

  /**
   * @brief Sets new position
   * @param position New position
   */
  void SetPosition(Eigen::VectorXd position) {
    position_ = std::move(position);
  }

  /**
   * @brief Returns ID of the landmark node
   * @return ID
   */
  [[nodiscard]] size_t GetID() const noexcept { return id_; }

 private:
  size_t id_{0};
  Eigen::VectorXd position_;
};

}  // namespace prime_slam::factor_graph
