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
#include <memory>

#include <Eigen/Core>

#include "prime_slam/observation/observation_batch.h"
#include "prime_slam/sensor/rgbd.h"

namespace prime_slam {
using observation::ObservationBatch;
using observation::ObservationTag;

/**
 * @brief Represents the position of the system in space and the observations at
 * that position
 * @tparam ObservationType Type of observation
 */
template <std::derived_from<ObservationTag> ObservationType>
class Frame final {
 public:
  /**
   * @brief Default constructor
   */
  Frame(size_t id = 0) : id_{id} {}

  /**
   * @brief Constructs a frame based on given observations, pose and camera
   * configuration
   * @param id Keyframe identifier
   * @param observations Set of observations (key objects)
   * @param sensor_data Data from sensors corresponding to current system pose
   * @param intrinsics Intrinsics of camera
   * @param world_to_camera_transform Transformation from the world coordinate
   * system to the camera coordinate system corresponding to the frame
   * @param is_keyframe Whether the frame is a keyframe
   */
  Frame(size_t id, ObservationBatch<ObservationType> observations,
        sensor::RGBDImage sensor_data, Eigen::Projective3d intrinsics,
        Eigen::Projective3d world_to_camera_transform, bool is_keyframe = false)
      : id_(id),
        observations_(std::make_unique<ObservationBatch<ObservationType>>(
            std::move(observations))),
        sensor_data_(std::move(sensor_data)),
        intrinsics_(std::move(intrinsics)),
        world_to_camera_transform_(std::move(world_to_camera_transform)),
        is_keyframe_(is_keyframe) {}

  Frame(size_t id, ObservationBatch<ObservationType> observations,
        sensor::RGBDImage sensor_data, const Eigen::Projective3d& intrinsics,
        bool is_keyframe = false)
      : Frame(id, std::move(observations), std::move(sensor_data), intrinsics,
              Eigen::Projective3d::Identity(), is_keyframe) {}

  Frame(ObservationBatch<ObservationType> observations)
      : Frame(/*id=*/0, /*observations=*/std::move(observations),
              /*sensor_data=*/{},
              /*intrinsics=*/Eigen::Projective3d::Identity()) {}

  /**
   * @brief Returns the set of observations corresponding to the frame
   * @return Set of observations
   */
  const ObservationBatch<ObservationType>& GetObservations() const& noexcept {
    return *observations_;
  }

  /**
   * @brief Returns the RGBD image corresponding to the frame
   * @return RGBD image
   */
  const sensor::RGBDImage& GetSensorData() const& noexcept {
    return sensor_data_;
  }

  /**
   * @brief Returns the camera intrinsics corresponding to the frame
   * @return Camera intrinsics
   */
  const Eigen::Projective3d& GetIntrinsics() const& noexcept {
    return intrinsics_;
  }

  /**
   * @brief Returns the Transformation from the world coordinate system to the
   * camera coordinate system corresponding to the frame
   * @return Transformation from the world to the camera
   */
  const Eigen::Projective3d& GetWorldToCameraTransform() const& noexcept {
    return world_to_camera_transform_;
  }

  /**
   * @brief Returns the origin of the camera coordinate system, represented in
   * the world coordinate system
   * @return Origin vector
   */
  [[nodiscard]] Eigen::Vector3d GetOrigin() const {
    return -(world_to_camera_transform_.rotation().matrix() *
             world_to_camera_transform_.inverse().translation().matrix());
  }

  /**
   * @brief Returns identifier of the frame
   * @return Frame id
   */
  [[nodiscard]] size_t GetID() const noexcept { return id_; }

  /**
   * @brief Returns whether a frame is a keyframe
   * @return True if the frame is a key, false otherwise
   */
  [[nodiscard]] bool IsKeyframe() const noexcept { return is_keyframe_; }

  /**
   * @brief Sets the is_keyframe value
   * @param is_keyframe Whether the frame is a keyframe
   */
  void SetKeyframe(bool is_keyframe = true) noexcept {
    is_keyframe_ = is_keyframe;
  }

  /**
   * @brief Sets a new pose value for the frame
   * @param new_pose New pose value
   */
  void UpdatePose(const Eigen::Projective3d& new_pose) {
    world_to_camera_transform_ = new_pose;
  }

 private:
  size_t id_;
  std::unique_ptr<ObservationBatch<ObservationType>> observations_;
  sensor::RGBDImage sensor_data_;
  Eigen::Projective3d intrinsics_;
  Eigen::Projective3d world_to_camera_transform_;
  bool is_keyframe_;
};

}  // namespace prime_slam
