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
#include <concepts>
#include <list>
#include <optional>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/range/combine.hpp>

#include "prime_slam/observation/observation_creator_concept.h"
#include "prime_slam/slam/frame/keyframe_selection/keyframe_selector_concept.h"
#include "prime_slam/slam/mapping/mapper.h"
#include "prime_slam/slam/mapping/mapper_concept.h"
#include "prime_slam/slam/tracking/tracker.h"
#include "prime_slam/slam/tracking/tracking_result.h"

namespace prime_slam::frontend {

using factor_graph::FactorGraph;
using mapping::DefaultMap;
using mapping::DefaultMapper;
using mapping::Map;
using mapping::Mapper;
using observation::ObservationCreator;
using tracking::DefaultTracker;
using tracking::Tracker;

/**
 * @brief Frontend based on tracking sequential keyframes
 * @tparam ObservationCreator Object that creates observations for sensor data
 * @tparam MapType Specific type of landmark map
 * @tparam KeyframeSelectorType Object that determines whether the new frame is
 * a frame key
 * @tparam TrackerType Specific type of tracker
 * @tparam MapperType Specific type of mapper
 */
template <ObservationCreator ObservationCreator,
          Map MapType =
              DefaultMap<typename ObservationCreator::ObservationType>,
          KeyframeSelector<typename ObservationCreator::ObservationType>
              KeyframeSelectorType = EveryNthKeyframeSelector<
                  typename ObservationCreator::ObservationType>,
          Tracker<MapType> TrackerType = DefaultTracker<MapType>,
          Mapper<MapType> MapperType = DefaultMapper<MapType>>
class TrackingFrontend {
 public:
  using ObservationType = typename ObservationCreator::ObservationType;
  using FrameType = Frame<ObservationType>;
  /**
   * @brief Constructs TrackingFrontend object
   * @param initial_pose Pose for the first keyframe
   * @param intrinsics Intrinsics of camera
   * @param creator Observation creator object
   * @param keyframe_selector Keyframe selector object
   * @param tracker Specific tracker
   * @param mapper Specific mapper
   */
  TrackingFrontend(Eigen::Projective3d initial_pose,
                   Eigen::Projective3d intrinsics, ObservationCreator creator,
                   KeyframeSelectorType keyframe_selector = {},
                   TrackerType tracker = {}, MapperType mapper = {});

  /**
   * @brief Takes a sensor data and returns true if it's inserted to SLAM
   * problem
   * @param sensor_data RGBD image
   * @return true if data was inserted
   */
  bool TryInsertSensorData(sensor::RGBDImage sensor_data);

  /**
   * @brief Returns true if factor graph optimization is required
   * @return is optimization required
   */
  bool OptimizationRequired() const noexcept;

  /**
   * @brief Returns the trajectory of frames
   * @return Absolute poses (as Eigen::Projective3d) for each keyframe
   */
  std::vector<Eigen::Projective3d> GetTrajectory() const;

  /**
   * @brief Returns the current map
   * @return Current map
   */
  const MapType& GetMap() const& noexcept { return mapper_.GetMap(); }

  /**
   * @brief Returns current factor graph representation
   * @return factor graph
   */
  const FactorGraph& GetGraph() const& noexcept { return graph_; }

  /**
   * @brief Updates the graph with backend-optimized data
   * @param result backend-optimized data
   */
  void UpdateGraph(backend::BackendResult result);

 private:
  void Initialize(FrameType keyframe);

  void InsertNewKeyframe(FrameType keyframe,
                         const DataAssociation& map_associations);

  size_t GetNewId() noexcept { return frame_counter_++; }

 private:
  using iterator = typename std::list<FrameType>::iterator;  // NOLINT
  size_t frame_counter_{0};
  Eigen::Projective3d initial_pose_;
  ObservationCreator observation_creator_;
  KeyframeSelectorType keyframe_selector_;
  TrackerType tracker_;
  MapperType mapper_;
  // keyframes stored in reverse order
  std::list<FrameType> reversed_keyframes_;
  // for quick access by keyframe id
  std::unordered_map<size_t, iterator> id_to_keyframe_;
  Eigen::Projective3d intrinsics_;
  FactorGraph graph_;
};

// IMPLEMENTATION

template <ObservationCreator ObservationCreator, Map MapType,
          KeyframeSelector<typename ObservationCreator::ObservationType>
              KeyframeSelectorType,
          Tracker<MapType> TrackerType, Mapper<MapType> MapperType>
inline TrackingFrontend<
    ObservationCreator, MapType, KeyframeSelectorType, TrackerType,
    MapperType>::TrackingFrontend(Eigen::Projective3d initial_pose,
                                  Eigen::Projective3d intrinsics,
                                  ObservationCreator creator,
                                  KeyframeSelectorType keyframe_selector,
                                  TrackerType tracker, MapperType mapper)
    : initial_pose_(std::move(initial_pose)),
      intrinsics_(std::move(intrinsics)),
      observation_creator_(std::move(creator)),
      keyframe_selector_(std::move(keyframe_selector)),
      tracker_(std::move(tracker)),
      mapper_(std::move(mapper)) {}

template <ObservationCreator ObservationCreator, Map MapType,
          KeyframeSelector<typename ObservationCreator::ObservationType>
              KeyframeSelectorType,
          Tracker<MapType> TrackerType, Mapper<MapType> MapperType>
inline bool TrackingFrontend<
    ObservationCreator, MapType, KeyframeSelectorType, TrackerType,
    MapperType>::TryInsertSensorData(sensor::RGBDImage sensor_data) {
  auto frame_id = GetNewId();
  auto observations = observation_creator_.Create(sensor_data);
  auto frame = Frame(frame_id, std::move(observations), std::move(sensor_data),
                     intrinsics_);
  bool inserted = false;
  if (frame_id == 0) {
    Initialize(std::move(frame));
    inserted = true;
  } else {
    auto relative_tracking_result =
        tracker_.Track(reversed_keyframes_.front(), frame);
    frame.UpdatePose(relative_tracking_result.pose);
    auto visible_map = mapper_.GetVisibleMap(frame);
    auto map_tracking_result = tracker_.TrackMap(frame, visible_map);
    frame.UpdatePose(map_tracking_result.pose);

    if (keyframe_selector_.IsSelected(frame)) {
      frame.SetKeyframe();
      InsertNewKeyframe(std::move(frame), map_tracking_result.associations);
      inserted = true;
    }
  }

  return inserted;
}

template <ObservationCreator ObservationCreator, Map MapType,
          KeyframeSelector<typename ObservationCreator::ObservationType>
              KeyframeSelectorType,
          Tracker<MapType> TrackerType, Mapper<MapType> MapperType>
inline bool TrackingFrontend<ObservationCreator, MapType, KeyframeSelectorType,
                             TrackerType, MapperType>::OptimizationRequired()
    const noexcept {
  return id_to_keyframe_.size() > 1;
}

template <ObservationCreator ObservationCreator, Map MapType,
          KeyframeSelector<typename ObservationCreator::ObservationType>
              KeyframeSelectorType,
          Tracker<MapType> TrackerType, Mapper<MapType> MapperType>
inline void
TrackingFrontend<ObservationCreator, MapType, KeyframeSelectorType, TrackerType,
                 MapperType>::Initialize(FrameType keyframe) {
  keyframe.UpdatePose(initial_pose_);
  keyframe.SetKeyframe();
  auto&& keyobjects_coordinates = keyframe.GetObservations().GetCoordinates();
  auto keyobject_to_landmark = mapper_.Initialize(keyframe);
  auto&& sensor_data = keyframe.GetSensorData();
  auto&& keyframe_id = keyframe.GetID();
  graph_.AddPoseNode(keyframe_id, keyframe.GetWorldToCameraTransform());

  for (auto&& [keyobject_id, landmark_id] : keyobject_to_landmark) {
    graph_.AddLandmarkNode(landmark_id, mapper_.At(landmark_id).GetPosition());
    graph_.AddObservationFactor(keyframe_id, landmark_id,
                                keyobjects_coordinates[keyobject_id],
                                sensor_data);
  }
  reversed_keyframes_.push_front(std::move(keyframe));
  id_to_keyframe_[keyframe_id] = reversed_keyframes_.begin();
}

template <ObservationCreator ObservationCreator, Map MapType,
          KeyframeSelector<typename ObservationCreator::ObservationType>
              KeyframeSelectorType,
          Tracker<MapType> TrackerType, Mapper<MapType> MapperType>
inline void TrackingFrontend<
    ObservationCreator, MapType, KeyframeSelectorType, TrackerType,
    MapperType>::InsertNewKeyframe(FrameType keyframe,
                                   const DataAssociation& map_associations) {
  auto keyframe_id = keyframe.GetID();
  graph_.AddPoseNode(keyframe_id, keyframe.GetWorldToCameraTransform());
  auto&& keyobjects_coordinates = keyframe.GetObservations().GetCoordinates();
  auto&& new_keyobjects_indices = map_associations.reference_indices;
  auto&& map_indices = map_associations.target_indices;

  for (auto&& [keyobject_index, map_index] :
       boost::combine(new_keyobjects_indices, map_indices)) {
    graph_.AddObservationFactor(keyframe_id, map_index,
                                keyobjects_coordinates[keyobject_index],
                                keyframe.GetSensorData());
  }

  auto keyobject_to_landmark =
      mapper_.MapFrameKeyobjects(keyframe, map_associations);

  for (auto&& [keyobject_id, landmark_id] : keyobject_to_landmark) {
    auto&& keyobject_coordinates = keyobjects_coordinates[keyobject_id];
    graph_.AddLandmarkNode(landmark_id, mapper_.At(landmark_id).GetPosition());
    graph_.AddObservationFactor(keyframe.GetID(), landmark_id,
                                keyobject_coordinates,
                                keyframe.GetSensorData());
  }
  reversed_keyframes_.push_front(std::move(keyframe));
  id_to_keyframe_[keyframe_id] = reversed_keyframes_.begin();
}

template <ObservationCreator ObservationCreator, Map MapType,
          KeyframeSelector<typename ObservationCreator::ObservationType>
              KeyframeSelectorType,
          Tracker<MapType> TrackerType, Mapper<MapType> MapperType>
inline std::vector<Eigen::Projective3d>
TrackingFrontend<ObservationCreator, MapType, KeyframeSelectorType, TrackerType,
                 MapperType>::GetTrajectory() const {
  std::vector<Eigen::Projective3d> trajectory;
  trajectory.reserve(reversed_keyframes_.size());
  std::transform(reversed_keyframes_.rbegin(), reversed_keyframes_.rend(),
                 std::back_inserter(trajectory),
                 [](auto&& item) { return item.GetWorldToCameraTransform(); });
  return trajectory;
}

template <ObservationCreator ObservationCreator, Map MapType,
          KeyframeSelector<typename ObservationCreator::ObservationType>
              KeyframeSelectorType,
          Tracker<MapType> TrackerType, Mapper<MapType> MapperType>
inline void
TrackingFrontend<ObservationCreator, MapType, KeyframeSelectorType, TrackerType,
                 MapperType>::UpdateGraph(backend::BackendResult result) {
  for (auto&& optimized_pose : result.optimized_poses) {
    id_to_keyframe_[optimized_pose.id]->UpdatePose(optimized_pose.pose);
    graph_.UpdatePose(optimized_pose.id, optimized_pose.pose);
  }
  for (auto&& optimized_position : result.optimized_positions) {
    mapper_.UpdateLandmarkPosition(optimized_position.id,
                                   optimized_position.position);
    graph_.UpdateLandmarkPosition(optimized_position.id,
                                  optimized_position.position);
  }
}
}  // namespace prime_slam::frontend
