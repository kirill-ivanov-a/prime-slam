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

#include "prime_slam/sensor/rgbd.h"

namespace prime_slam::factor_graph {

/**
 * @brief Represent stereo observation in factor graph
 */
class StereoObservationFactor final {
 public:
  /**
   * @brief Creates a factor (correspondence) between the observation
   * (keyobject) from the frame and the landmark
   * @param keyobject_position Position of the keyobject in the image
   * @param pose_node_id ID of the corresponding pose node
   * @param landmark_node_id ID of the corresponding landmark node
   * @param sensor_data Sensor data on which the key object was detected
   * @param bf To simulate stereo viewing, the size of the baseline multiplied
   * by the focal length for x
   */
  StereoObservationFactor(const Eigen::Vector2d& keyobject_position,
                          size_t pose_node_id, size_t landmark_node_id,
                          const sensor::RGBDImage& sensor_data,
                          double bf = 400.0)
      : stereo_observation_(
            ConvertToStereo(keyobject_position, sensor_data.GetDepth(), bf)),
        pose_node_id_(pose_node_id),
        landmark_node_id_(landmark_node_id) {}

  StereoObservationFactor(Eigen::Vector3d stereo_observation,
                          size_t pose_node_id, size_t landmark_node_id)
      : stereo_observation_(std::move(stereo_observation)),
        pose_node_id_(pose_node_id),
        landmark_node_id_(landmark_node_id) {}

  /**
   * @brief Returns observation at the corresponding keyframe
   * @return Observation position
   */
  [[nodiscard]] Eigen::VectorXd GetObservation() const {
    return stereo_observation_;
  }

  /**
   * @brief Returns corresponding pose node ID
   * @return Pose Node ID
   */
  [[nodiscard]] size_t GetPoseNodeID() const noexcept { return pose_node_id_; }

  /**
   * @brief Returns corresponding landmark node ID
   * @return Landmark Node ID
   */
  [[nodiscard]] size_t GetLandmarkNodeID() const noexcept {
    return landmark_node_id_;
  }

 private:
  static Eigen::Vector3d ConvertToStereo(const Eigen::Vector2d& coordinates,
                                         const Eigen::MatrixXd& depth_map,
                                         double bf) {
    auto x = static_cast<int>(coordinates[0]);
    auto y = static_cast<int>(coordinates[1]);
    auto depth = depth_map(y, x);
    return Eigen::Vector3d{static_cast<double>(x), static_cast<double>(y),
                           x - bf / depth};
  }

 private:
  Eigen::Vector3d stereo_observation_;
  size_t pose_node_id_{0};
  size_t landmark_node_id_{0};
};

}  // namespace prime_slam::factor_graph
