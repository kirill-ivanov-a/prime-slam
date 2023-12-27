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
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "prime_slam/observation/traits/observation_traits.h"
#include "prime_slam/slam/frame/frame.h"

namespace prime_slam::mapping {

using observation::ObservationTag;
using observation::ObservationTraits;

template <std::derived_from<ObservationTag> ObservationType>
using LandmarkCoordinatesType =
    Eigen::Vector<double,
                  ObservationTraits<ObservationType>::LandmarkDimension>;

/**
 * @brief Map concept
 * @tparam MapImpl Concrete map
 */
template <typename MapImpl>
concept Map = std::ranges::range<MapImpl> &&
    std::derived_from<typename MapImpl::ObservationType, ObservationTag> &&
    requires(MapImpl map) {
  // returns map landmark positions
  {
    map.GetPositions()
    } -> std::convertible_to<std::vector<
        LandmarkCoordinatesType<typename MapImpl::ObservationType>>>;
  // returns map landmark descriptors
  { map.GetDescriptors() } -> std::convertible_to<std::vector<Eigen::VectorXd>>;
  // removes specific landmarks by IDs
  {map.RemoveLandmarks(std::declval<std::vector<size_t>>())};
  // returns landmarks IDs
  { map.GetIDs() } -> std::convertible_to<std::vector<size_t>>;
  // returns map size
  { map.Size() } -> std::convertible_to<size_t>;
  // inserts new Landmark
  {map.InsertLandmark(std::declval<Landmark>())};
  // sets a new position to a landmark with the corresponding ID
  {map.UpdateLandmarkPosition(std::declval<size_t>(),
                              std::declval<Eigen::VectorXd>())};
  // returns landmark by ID
  { map.At(std::declval<size_t>()) } -> std::convertible_to<Landmark>;
  // Returns map visible from frame
  {
    map.GetVisibleMap(std::declval<Frame<typename MapImpl::ObservationType>>())
    } -> std::convertible_to<MapImpl>;
};

}  // namespace prime_slam::mapping
