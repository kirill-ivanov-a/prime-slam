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
#include <memory>
#include <vector>

#include "prime_slam/observation/keyobject_concept.h"

namespace prime_slam::observation {

/**
 * @brief Concept of keyobject descriptor
 * @tparam DescriptorImpl Concrete descriptor type
 * @tparam ImageType Type of image using for description
 * @tparam KeyobjectType Type of keyobject to descript
 */
template <typename DescriptorImpl, typename ImageType, typename KeyobjectType>
concept Descriptor = requires(DescriptorImpl descriptor) {
  {
    // Returns vector of descriptors
    descriptor.Descript(std::declval<std::vector<KeyobjectType>>(),
                        std::declval<ImageType>())
    } -> std::convertible_to<std::vector<Eigen::VectorXd>>;
};

}  // namespace prime_slam::observation
