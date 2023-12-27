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

#include <list>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/range/adaptor/reversed.hpp>
#include <Eigen/Core>

#include "prime_slam/observation/traits/observation_traits.h"
#include "prime_slam/slam/mapping/landmark.h"
#include "prime_slam/slam/mapping/map_concept.h"
#include "prime_slam/slam/mapping/visible_landmarks_mask.h"

namespace prime_slam::mapping {

using observation::ObservationTraits;

/**
 * @brief Represent landmark map of the given observation type
 * @tparam Observation Concrete type of observation
 */
template <std::derived_from<observation::ObservationTag> Observation>
class DefaultMap {
 public:
  using ObservationType = Observation;  // NOLINT
  using LandmarkCoordinatesType =
      Eigen::Vector<double,
                    ObservationTraits<ObservationType>::LandmarkDimension>;
  using iterator = typename std::list<Landmark>::iterator;  // NOLINT
  using const_itarator =                                    // NOLINT
      typename std::list<Landmark>::const_iterator;

 public:
  /**
   * @brief Constructs empty map
   */
  DefaultMap() = default;

  /**
   * @brief Returns positions of the map landmarks
   * @return Landmarks Positions
   */
  const std::vector<LandmarkCoordinatesType>& GetPositions() const& noexcept {
    return positions_;
  }

  /**
   * @brief Returns descriptors of the map landmarks
   * @return Landmarks Descriptors
   */
  const std::vector<Eigen::VectorXd>& GetDescriptors() const& noexcept {
    return descriptors_;
  }

  /**
   * @brief Returns IDs of the map landmarks
   * @return Landmarks IDs
   */
  const std::vector<size_t>& GetIDs() const& noexcept { return ids_; };

  /**
   * @brief Returns current map size
   * @return Number of the map landmarks
   */
  size_t Size() const noexcept { return reversed_landmarks_.size(); }

  /**
   * @brief Return landmarks with corresponding ID
   * @param landmark_id Desired landmark ID
   * @return Const reference to the landmark object
   */
  const Landmark& At(size_t landmark_id) const& {
    auto value = id_to_landmark_.at(landmark_id);
    return *value;
  }

  /**
   * @brief Remove landmarks with corresponding IDs
   * @param ids IDs of landmarks to remove
   */
  void RemoveLandmarks(const std::vector<size_t>& ids);

  /**
   * @brief Inserts new landmark
   * @param landmark Landmark to insert
   */
  void InsertLandmark(Landmark landmark);

  /**
   * @brief Sets a new position to a landmark with the corresponding ID
   * @param landmark_id ID of the landmark to update
   * @param position New position
   */
  void UpdateLandmarkPosition(size_t landmark_id, Eigen::Vector3d position) {
    id_to_landmark_[landmark_id]->SetPosition(std::move(position));
  }

  /**
   * @brief Returns begin map iterator
   * @return Begin map iterator
   */
  iterator begin() noexcept {  // NOLINT
    return reversed_landmarks_.begin();
  }

  /**
   * @brief Returns end map iterator
   * @return End map iterator
   */
  iterator end() noexcept {  // NOLINT
    return reversed_landmarks_.end();
  }

  /**
   * @brief Returns constant begin map iterator
   * @return Const begin map iterator
   */
  const_itarator begin() const noexcept {  // NOLINT
    return reversed_landmarks_.cbegin();
  }

  /**
   * @brief Returns constant end map iterator
   * @return Const end map iterator
   */
  const_itarator end() const noexcept {  // NOLINT
    return reversed_landmarks_.cend();
  }

  /**
   * @brief Returns constant begin map iterator
   * @return Const begin map iterator
   */
  const_itarator cbegin() const noexcept {  // NOLINT
    return reversed_landmarks_.cbegin();
  }

  /**
   * @brief Returns constant end map iterator
   * @return Const end map iterator
   */
  const_itarator cend() const noexcept {  // NOLINT
    return reversed_landmarks_.cend();
  }

  /**
   * @brief Returns map visible from frame
   * @param frame Frame from which the map is observed
   * @return Part of the map (another map) that is visible from the frame
   */
  DefaultMap GetVisibleMap(const Frame<ObservationType>& frame) const;

 private:
  void RecalculateInternals();

 private:
  // Due to culling, landmarks must be removed frequently.
  // A list is better suited for these purposes than a vector
  std::list<Landmark> reversed_landmarks_;
  // For quick access to the Landmark by ID
  std::unordered_map<size_t, iterator> id_to_landmark_;
  std::vector<LandmarkCoordinatesType> positions_;
  std::vector<Eigen::VectorXd> descriptors_;
  std::vector<size_t> ids_;
};

// IMPLEMENTATION

template <std::derived_from<observation::ObservationTag> Observation>
inline void DefaultMap<Observation>::RemoveLandmarks(
    const std::vector<size_t>& ids) {
  for (auto id : ids) {
    auto it = id_to_landmark_.find(id);
    if (it != id_to_landmark_.end()) {
      reversed_landmarks_.erase(it->second);
      id_to_landmark_.erase(it);
    }
  }
  RecalculateInternals();
}

template <std::derived_from<observation::ObservationTag> Observation>
inline void DefaultMap<Observation>::InsertLandmark(Landmark landmark) {
  auto id = landmark.GetID();
  positions_.push_back(landmark.GetPosition());
  descriptors_.push_back(landmark.GetDescriptor());
  ids_.push_back(id);
  reversed_landmarks_.emplace_front(std::move(landmark));
  id_to_landmark_.insert({id, reversed_landmarks_.begin()});
}

template <std::derived_from<observation::ObservationTag> Observation>
inline DefaultMap<Observation> DefaultMap<Observation>::GetVisibleMap(
    const Frame<ObservationType>& frame) const {
  auto mask = VisibleLandmarksMask<ObservationType>::Create(positions_, frame);
  auto visible_map = DefaultMap{};
  // Landmarks stored in reverse order
  // Create reversed view
  auto landmarks = reversed_landmarks_ | boost::adaptors::reversed;
  for (auto&& [landmark, visible] : boost::combine(landmarks, mask)) {
    if (visible) {
      visible_map.InsertLandmark(landmark);
    }
  }

  return visible_map;
}

template <std::derived_from<observation::ObservationTag> Observation>
inline void DefaultMap<Observation>::RecalculateInternals() {
  positions_.clear();
  descriptors_.clear();
  ids_.clear();
  auto landmarks = reversed_landmarks_ | boost::adaptors::reversed;
  for (auto&& landmark : landmarks) {
    positions_.push_back(landmark.GetPosition());
    descriptors_.push_back(landmark.GetDescriptor());
    ids_.push_back(landmark.GetID());
  }
}

}  // namespace prime_slam::mapping
