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

#include <concepts>
#include <cstddef>

#include "prime_slam/observation/traits/observation_tag.h"

namespace prime_slam::observation {

/**
 * @brief Traits of observations
 * @tparam Observation Concrete Observation Tag
 */
template <std::derived_from<ObservationTag> Observation>
struct ObservationTraits;

/**
 * @brief Traits for Point Observation
 */
template <>
struct ObservationTraits<PointObservationTag> {
  static constexpr const size_t KeyobjectDimension = 2;
  static constexpr const size_t LandmarkDimension = 3;
};

/**
 * @brief Traits for Line Observation
 */
template <>
struct ObservationTraits<LineObservationTag> {
  static constexpr const size_t KeyobjectDimension = 4;
  static constexpr const size_t LandmarkDimension = 6;
};

}  // namespace prime_slam::observation
