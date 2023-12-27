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

#include <Eigen/Core>
#include <vector>

#include "prime_slam/observation/matching/match.h"

namespace prime_slam::observation {

using Descriptors = std::vector<Eigen::VectorXd>;

/**
 * @brief Concept of descriptors matcher
 * @tparam MatcherImpl Concrete matcher
 */
template <typename MatcherImpl>
concept Matcher = std::movable<MatcherImpl> && requires(MatcherImpl matcher) {
  {
    // Matches two sets of descriptors
    matcher.MatchDescriptors(std::declval<Descriptors>(),
                             std::declval<Descriptors>())
    } -> std::convertible_to<std::vector<Match>>;
};

}  // namespace prime_slam::observation
