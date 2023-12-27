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

#include <unordered_map>
#include <utility>

#include <boost/range/combine.hpp>

#include "prime_slam/slam/data_association.h"
#include "prime_slam/slam/mapping/landmark.h"
#include "prime_slam/slam/mapping/map.h"
#include "prime_slam/slam/mapping/map_concept.h"
#include "prime_slam/slam/mapping/mapper_concept.h"

namespace prime_slam::mapping {

/**
 * @brief Represents an object that maps new observations to a map
 * @tparam MapType Concrete map type that implements Map concept
 */
template <Map MapType>
class DefaultMapper {
 public:
  using ObservationType = typename MapType::ObservationType;
  using FrameType = Frame<ObservationType>;

 public:
  /**
   * @brief Initializes map using initial keyframe observations
   * @param initial_keyframe First keyframe in sequence
   * @return Matching the indices of keyobjects and IDs of landmarks
   */
  std::unordered_map<size_t, size_t> Initialize(
      const FrameType& initial_keyframe);

  /**
   * @brief Maps keyframe observations to the map
   * @param keyframe Keyframe whose observations should be reflected in the map
   * @param keyobject_to_map_associations Associations between key objects and
   * landmarks of the map
   * @return Matching the indices of the new mapped key objects and landmark
   * identifiers
   */
  std::unordered_map<size_t, size_t> MapFrameKeyobjects(
      const FrameType& keyframe,
      const DataAssociation& keyobject_to_map_associations);

  /**
   * @brief Sets a new position to a landmark with the corresponding ID
   * @param landmark_id ID of the landmark to update
   * @param position New position
   */
  void UpdateLandmarkPosition(size_t landmark_id,
                              const Eigen::Vector3d& position) {
    map_.UpdateLandmarkPosition(landmark_id, position);
  }

  /**
   * @brief Returns landmark with corresponding ID
   * @param landmark_id Desired landmark ID
   * @return Const reference to the landmark object
   */
  [[nodiscard]] const Landmark& At(size_t landmark_id) const& {
    return map_.At(landmark_id);
  }

  /**
   * @brief Returns map visible from frame
   * @param frame Frame from which the map is observed
   * @return Part of the map (another map) that is visible from the frame
   */
  MapType GetVisibleMap(const FrameType& frame) const {
    return map_.GetVisibleMap(frame);
  }

  /**
   * @brief Returns current map
   * @return Current map
   */
  const MapType& GetMap() const& { return map_; }

 private:
  size_t GetNewID() noexcept { return landmarks_counter_++; }

 private:
  DefaultMap<ObservationType> map_;
  projection::DefaultProjector<ObservationType> projector_;
  size_t landmarks_counter_{0};
};

// IMPLEMENTATION

template <Map MapType>
inline std::unordered_map<size_t, size_t> DefaultMapper<MapType>::Initialize(
    const DefaultMapper::FrameType& initial_keyframe) {
  auto&& observations = initial_keyframe.GetObservations();
  auto&& keyobjects_descriptors = observations.GetDescriptors();
  auto keyobjects_coordinates = observations.GetCoordinates();
  auto keyobjects_number = observations.Size();
  auto landmarks_coordinates = projector_.BackProject(
      keyobjects_coordinates, initial_keyframe.GetIntrinsics(),
      initial_keyframe.GetSensorData().GetDepth(),
      initial_keyframe.GetWorldToCameraTransform());

  std::unordered_map<size_t, size_t> keyobject_to_landmark;
  keyobject_to_landmark.reserve(keyobjects_coordinates.size());

  for (auto keyobject_id = 0; keyobject_id != keyobjects_number;
       ++keyobject_id) {
    auto landmark_id = GetNewID();
    keyobject_to_landmark.emplace(keyobject_id, landmark_id);
    map_.InsertLandmark(Landmark{landmark_id,
                                 landmarks_coordinates[keyobject_id],
                                 keyobjects_descriptors[keyobject_id]});
  }

  return keyobject_to_landmark;
}

template <Map MapType>
inline std::unordered_map<size_t, size_t>
DefaultMapper<MapType>::MapFrameKeyobjects(
    const DefaultMapper::FrameType& keyframe,
    const DataAssociation& keyobject_to_map_associations) {
  auto&& keyobjects_descriptors = keyframe.GetObservations().GetDescriptors();
  auto&& keyobjects_coordinates = keyframe.GetObservations().GetCoordinates();
  auto&& unmatched_keypoints_indices =
      keyobject_to_map_associations.unmatched_reference_indices;

  std::unordered_map<size_t, size_t> keyobject_to_landmark;
  keyobject_to_landmark.reserve(unmatched_keypoints_indices.size());
  for (auto unmatched_index : unmatched_keypoints_indices) {
    auto&& landmark_coordinates = projector_.BackProject(
        keyobjects_coordinates[unmatched_index], keyframe.GetIntrinsics(),
        keyframe.GetSensorData().GetDepth(),
        keyframe.GetWorldToCameraTransform());
    // landmark descriptor = associated keyobject descriptor
    auto&& descriptor = keyobjects_descriptors[unmatched_index];
    auto landmark_id = GetNewID();
    auto landmark = Landmark{landmark_id, landmark_coordinates, descriptor};
    keyobject_to_landmark.emplace(unmatched_index, landmark_id);
    map_.InsertLandmark(std::move(landmark));
  }

  return keyobject_to_landmark;
}

}  // namespace prime_slam::mapping
