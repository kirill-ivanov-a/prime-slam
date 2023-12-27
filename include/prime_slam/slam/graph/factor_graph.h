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

#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "factor/stereo_observation_factor.h"
#include "node/landmark_node.h"
#include "node/pose_node.h"

namespace prime_slam::factor_graph {

/**
 * @brief Factor graph representing the current configuration of the SLAM system
 *
 * Contains landmark nodes that correspond to the map's landmarks, poses that
 * correspond to the trajectory of the keyframes, as well as factors
 * representing the observations of the landmarks from the keyframes
 */
class FactorGraph final {
 public:
  /**
   * @brief Default constructor
   */
  FactorGraph() = default;

  /**
   * @brief Adds new pose node to the graph
   * @param frame_id Corresponding frame id
   * @param pose Pose of the corresponding frame
   */
  void AddPoseNode(size_t frame_id, Eigen::Projective3d pose) {
    pose_nodes_.emplace_front(frame_id, std::move(pose));
    pose_id_to_node_.insert({frame_id, pose_nodes_.begin()});
  }

  /**
   * @brief Adds new landmark node to the graph
   * @param landmark_id Corresponding landmark id
   * @param landmark_position Position of the corresponding landmark
   */
  void AddLandmarkNode(size_t landmark_id, Eigen::VectorXd landmark_position) {
    landmark_nodes_.emplace_front(landmark_id, std::move(landmark_position));
    landmark_id_to_node_.insert({landmark_id, landmark_nodes_.begin()});
  }

  /**
   * @brief Adds a factor (correspondence) between the observation (keyobject)
   * from the frame and the landmark
   * @param pose_id ID of the corresponding pose node
   * @param landmark_id ID of the corresponding landmark node
   * @param keyobject_position Position of the keyobject in the image
   * @param sensor_data Sensor data on which the key object was detected
   */
  void AddObservationFactor(size_t pose_id, size_t landmark_id,
                            const Eigen::VectorXd& keyobject_position,
                            const sensor::RGBDImage& sensor_data) {
    observation_factors_.emplace_back(keyobject_position, pose_id, landmark_id,
                                      sensor_data);
  }

  /**
   * @brief Updates the pose for a pose node with the corresponding ID
   * @param pose_node_id Pose id to update
   * @param new_pose New pose
   */
  void UpdatePose(size_t pose_node_id, const Eigen::Projective3d& new_pose) {
    pose_id_to_node_[pose_node_id]->SetPose(new_pose);
  }

  /**
   * @brief Updates the position for a landmark node with the corresponding ID
   * @param landmark_node_id Pose id to update
   * @param new_position New position
   */
  void UpdateLandmarkPosition(size_t landmark_node_id,
                              const Eigen::VectorXd& new_position) {
    landmark_id_to_node_[landmark_node_id]->SetPosition(new_position);
  }

  /**
   * @brief Returns current pose nodes
   * @return Current pose nodes
   */
  const std::list<PoseNode>& GetPoseNodes() const& noexcept {
    return pose_nodes_;
  }

  /**
   * @brief Returns current landmark nodes
   * @return Current landmark nodes
   */
  const std::list<LandmarkNode>& GetLandmarkNodes() const& noexcept {
    return landmark_nodes_;
  }

  /**
   * @brief Returns current observation factors
   * @return Current observation factors
   */
  const std::vector<StereoObservationFactor>& GetObservationFactors()
      const& noexcept {
    return observation_factors_;
  }

 private:
  using PoseIterator = std::list<PoseNode>::iterator;
  using LandmarkIterator = std::list<LandmarkNode>::iterator;
  std::list<PoseNode> pose_nodes_;
  std::unordered_map<size_t, PoseIterator> pose_id_to_node_;
  std::list<LandmarkNode> landmark_nodes_;
  std::unordered_map<size_t, LandmarkIterator> landmark_id_to_node_;
  std::vector<StereoObservationFactor> observation_factors_;
};

}  // namespace prime_slam::factor_graph
