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

#include <memory>
#include <unordered_map>

#include "prime_slam/slam/data_association.h"
#include "prime_slam/slam/frame/frame.h"
#include "prime_slam/slam/mapping/map_concept.h"

namespace prime_slam::mapping {

/**
 * @brief Mapper concept
 * @tparam MapperImpl Concrete mapper type
 * @tparam MapImpl Concrete map type
 */
template <typename MapperImpl, typename MapImpl>
concept Mapper = Map<MapImpl> && requires(MapperImpl mapper, MapImpl map) {
  // initializes map using initial keyframe observations
  {
    mapper.Initialize(std::declval<Frame<typename MapImpl::ObservationType>>())
    } -> std::convertible_to<std::unordered_map<size_t, size_t>>;
  // maps keyframe observations to the map
  {
    mapper.MapFrameKeyobjects(
        std::declval<Frame<typename MapImpl::ObservationType>>(),
        std::declval<DataAssociation>())
    } -> std::convertible_to<std::unordered_map<size_t, size_t>>;
  // sets a new position to a landmark with the corresponding ID
  {mapper.UpdateLandmarkPosition(std::declval<size_t>(),
                                 std::declval<Eigen::VectorXd>())};
  // returns landmark with corresponding ID
  { mapper.At(std::declval<size_t>()) } -> std::convertible_to<Landmark>;
  // returns map visible from frame
  {
    mapper.GetVisibleMap(
        std::declval<Frame<typename MapImpl::ObservationType>>())
    } -> std::convertible_to<MapImpl>;
  // returns current map
  { mapper.GetMap() } -> std::convertible_to<MapImpl>;
};

}  // namespace prime_slam::mapping
