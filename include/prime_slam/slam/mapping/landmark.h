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

#include <utility>

#include <Eigen/Core>

namespace prime_slam::mapping {

/**
 * @brief Represents 3D object position in the world coordinate system
 */
class Landmark final {
 public:
  /**
   * @brief Default constructor
   */
  Landmark() = default;

  /**
   * @brief Creates landmark from given position, descriptor, and ID
   * @param id Unique ID of the landmark
   * @param position Position of the landmark
   * @param descriptor Descriptor of the landmark
   */
  Landmark(size_t id, Eigen::VectorXd position, Eigen::VectorXd descriptor)
      : id_(id),
        position_(std::move(position)),
        descriptor_(std::move(descriptor)) {}

 public:
  // getters

  /**
   * @brief Returns ID of the current landmark
   * @return Landmark ID
   */
  size_t GetID() const noexcept { return id_; }

  /**
   * @brief Returns position of the current landmark
   * @return Landmark Position
   */
  Eigen::Vector3d GetPosition() const { return position_; }

  /**
   * @brief Returns descriptor of the current landmark
   * @return Landmark Descriptor
   */
  const Eigen::VectorXd& GetDescriptor() const& noexcept { return descriptor_; }

 public:
  // setters

  /**
   * @brief Sets new position of the current landmark
   * @return New Position
   */
  void SetPosition(Eigen::Vector3d new_position) { position_ = new_position; }

 private:
  size_t id_{0};
  Eigen::VectorXd position_;
  Eigen::VectorXd descriptor_;
};

}  // namespace prime_slam::mapping
