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

#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "pose_estimation/pose_estimator_concept.h"
#include "pose_estimation/rgbd_point_pose_estimator.h"
#include "prime_slam/observation/matching/matcher_concept.h"
#include "prime_slam/observation/matching/opencv_matcher.h"
#include "prime_slam/projection/projector_concept.h"
#include "prime_slam/slam/data_association.h"
#include "prime_slam/slam/tracking/tracker_concept.h"
#include "prime_slam/slam/tracking/tracking_result.h"

namespace prime_slam::tracking {

using mapping::Map;
using observation::Matcher;
using observation::OpenCVMatcher;
using projection::DefaultProjector;
using projection::Projector;

/**
 * @brief Frame pose change tracker
 * @tparam MapType Concrete map type
 * @tparam MatcherType Observation Matcher
 * @tparam ProjectorType Observation Projector
 * @tparam PoseEstimatorType Pose Estimator
 */
template <Map MapType, Matcher MatcherType = OpenCVMatcher,
          Projector<typename MapType::ObservationType> ProjectorType =
              DefaultProjector<typename MapType::ObservationType>,
          PoseEstimator<typename MapType::ObservationType> PoseEstimatorType =
              RGBDPoseEstimator<typename MapType::ObservationType>>
class DefaultTracker {
 public:
  using ObservationType = typename MapType::ObservationType;
  using FrameType = Frame<ObservationType>;
  /**
   * @brief Creates Tracker object
   * @param matcher Matcher object
   * @param projector Projector object
   * @param estimator Estimator object
   */
  DefaultTracker(MatcherType matcher = {}, ProjectorType projector = {},
                 PoseEstimatorType estimator = {})
      : matcher_(std::move(matcher)),
        projector_(std::move(projector)),
        estimator_(std::move(estimator)) {}

  /**
   * @brief Tracks frame changes relative to the map
   * @param frame Frame to track
   * @param map Landmarks Map
   * @return Tracking result including the absolute pose of the frame and the
   * associations between the frame observations and the frame
   */
  [[nodiscard]] TrackingResult TrackMap(const FrameType& frame,
                                        const MapType& map) const;

  /**
   * @brief Tracks a new frame relative to the previous one
   * @param prev_frame Previous frame
   * @param new_frame New frame
   * @return Tracking result including the relative pose of the new frame with
   * respect to the previous one and the associations between the frames
   */
  [[nodiscard]] TrackingResult Track(const FrameType& prev_frame,
                                     const FrameType& new_frame) const;

 private:
  static std::vector<size_t> GetUnmatchedIndices(
      size_t indices_size, const std::vector<size_t>& matched_indices);

 private:
  MatcherType matcher_;
  ProjectorType projector_;
  PoseEstimatorType estimator_;
};

// IMPLEMENTATION

template <Map MapType, Matcher MatcherType,
          Projector<typename MapType::ObservationType> ProjectorType,
          PoseEstimator<typename MapType::ObservationType> PoseEstimatorType>
inline TrackingResult
DefaultTracker<MapType, MatcherType, ProjectorType, PoseEstimatorType>::Track(
    const FrameType& prev_frame, const FrameType& new_frame) const {
  auto&& new_frame_descriptors = new_frame.GetObservations().GetDescriptors();
  auto&& prev_frame_descriptors = prev_frame.GetObservations().GetDescriptors();
  auto matches =
      matcher_.MatchDescriptors(new_frame_descriptors, prev_frame_descriptors);

  auto prev_coords = prev_frame.GetObservations().GetCoordinates();
  auto prev_keypoints_3d = projector_.BackProject(
      prev_coords, prev_frame.GetIntrinsics(),
      prev_frame.GetSensorData().GetDepth(), Eigen::Projective3d::Identity());

  auto prev_to_new_relative_pose = estimator_.EstimatePose(
      new_frame.GetObservations().GetCoordinates(), new_frame.GetIntrinsics(),
      prev_keypoints_3d, matches);

  std::vector<size_t> reference_indices;
  std::vector<size_t> target_indices;
  reference_indices.reserve(matches.size());
  target_indices.reserve(matches.size());

  for (auto match : matches) {
    reference_indices.push_back(match.reference_index);
    target_indices.push_back(match.target_index);
  }

  auto unmatched_reference_indices =
      GetUnmatchedIndices(new_frame_descriptors.size(), reference_indices);
  auto unmatched_target_indices =
      GetUnmatchedIndices(prev_frame_descriptors.size(), target_indices);

  auto initial_absolute_pose =
      (prev_to_new_relative_pose * prev_frame.GetWorldToCameraTransform());

  auto data_associations =
      DataAssociation{std::move(reference_indices), std::move(target_indices),
                      std::move(unmatched_reference_indices),
                      std::move(unmatched_target_indices)};

  return TrackingResult{std::move(initial_absolute_pose),
                        std::move(data_associations)};
}

template <Map MapType, Matcher MatcherType,
          Projector<typename MapType::ObservationType> ProjectorType,
          PoseEstimator<typename MapType::ObservationType> PoseEstimatorType>
inline TrackingResult
DefaultTracker<MapType, MatcherType, ProjectorType,
               PoseEstimatorType>::TrackMap(const FrameType& frame,
                                            const MapType& map) const {
  auto&& initial_absolute_pose = frame.GetWorldToCameraTransform();
  auto&& landmark_positions = map.GetPositions();
  auto&& landmark_descriptors = map.GetDescriptors();
  auto&& landmark_ids = map.GetIDs();

  auto landmark_positions_cam =
      projector_.Transform(landmark_positions, initial_absolute_pose);
  auto frame_descriptors = frame.GetObservations().GetDescriptors();
  auto matches =
      matcher_.MatchDescriptors(frame_descriptors, landmark_descriptors);
  auto absolute_pose_delta = estimator_.EstimatePose(
      frame.GetObservations().GetCoordinates(), frame.GetIntrinsics(),
      landmark_positions_cam, matches);
  std::vector<size_t> reference_indices;
  std::vector<size_t> target_indices;
  reference_indices.reserve(matches.size());
  target_indices.reserve(matches.size());

  for (auto&& match : matches) {
    reference_indices.push_back(match.reference_index);
    target_indices.push_back(landmark_ids[match.target_index]);
  }
  auto unmatched_reference_indices =
      GetUnmatchedIndices(frame_descriptors.size(), reference_indices);
  auto unmatched_target_indices =
      GetUnmatchedIndices(landmark_descriptors.size(), target_indices);

  auto data_associations =
      DataAssociation{std::move(reference_indices), std::move(target_indices),
                      std::move(unmatched_reference_indices),
                      std::move(unmatched_target_indices)};

  auto absolute_pose = absolute_pose_delta * initial_absolute_pose;

  return TrackingResult{std::move(absolute_pose), std::move(data_associations)};
}

template <Map MapType, Matcher MatcherType,
          Projector<typename MapType::ObservationType> ProjectorType,
          PoseEstimator<typename MapType::ObservationType> PoseEstimatorType>
inline std::vector<size_t>
DefaultTracker<MapType, MatcherType, ProjectorType, PoseEstimatorType>::
    GetUnmatchedIndices(size_t indices_size,
                        const std::vector<size_t>& matched_indices) {
  std::vector<size_t> unmatched_indices;
  std::unordered_set<size_t> matched_indices_set(matched_indices.begin(),
                                                 matched_indices.end());
  unmatched_indices.reserve(indices_size - matched_indices.size());
  for (auto index = 0ul; index != indices_size; ++index) {
    if (matched_indices_set.find(index) == matched_indices_set.end()) {
      unmatched_indices.push_back(index);
    }
  }
  return unmatched_indices;
}

}  // namespace prime_slam::tracking
