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
#include <iterator>

#include <Eigen/Core>
#include <opencv2/features2d.hpp>

#include "prime_slam/observation/traits/observation_tag.h"
#include "prime_slam/observation/traits/observation_traits.h"

namespace prime_slam::observation {

template <typename KeyobjectImpl>
using CooridnatesType = Eigen::Vector<
    double, ObservationTraits<
                typename KeyobjectImpl::ObservationType>::KeyobjectDimension>;

/**
 * @brief Concept of key object
 * @tparam KeyobjectImpl Concrete key object type
 */
template <typename KeyobjectImpl>
concept Keyobject = std::movable<KeyobjectImpl> &&
    std::default_initializable<KeyobjectImpl> &&
    std::derived_from<typename KeyobjectImpl::ObservationType,
                      ObservationTag> && requires(KeyobjectImpl keyobject) {
  // Returns coordinates of key object in image
  {
    keyobject.GetCoordinates()
    } -> std::convertible_to<CooridnatesType<KeyobjectImpl>>;
  // Returns uncertainty associated with key object
  { keyobject.GetUncertainty() } -> std::floating_point;
};

}  // namespace prime_slam::observation
