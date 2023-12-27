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
#include <vector>

#include "prime_slam/observation/keyobject_concept.h"
#include "prime_slam/sensor/rgbd.h"

namespace prime_slam::observation {

/**
 * @brief Concept of keyobject filter
 * @tparam KeyobjectsFilterImpl Concrete keyobject filter
 * @tparam KeyobjectImpl Type of keyobject
 */
template <typename KeyobjectsFilterImpl, typename KeyobjectImpl>
concept KeyobjectsFilter = Keyobject<KeyobjectImpl> &&
    requires(KeyobjectsFilterImpl filter) {
  {
    // Returns filtered keyobjects
    filter.Apply(std::declval<std::vector<KeyobjectImpl>>(),
                 std::declval<sensor::RGBDImage>())
    } -> std::convertible_to<std::vector<KeyobjectImpl>>;
};

}  // namespace prime_slam::observation
