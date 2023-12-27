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

#include "prime_slam/slam/frame/frame.h"

namespace prime_slam {
/**
 * @brief Keyframe selector concept
 *
 * A frame selector is an object that takes a frame and determines whether it is
 * a keyframe
 * @tparam KeyframeSelectorImpl Concrete keyframe selector
 * @tparam ObservationType Type of observation
 */
template <typename KeyframeSelectorImpl, typename ObservationType>
concept KeyframeSelector =
    std::derived_from<ObservationType, observation::ObservationTag> &&
    requires(KeyframeSelectorImpl selector) {
  // Takes the frame and return true if it is the keyframe
  {
    selector.IsSelected(std::declval<Frame<ObservationType>>())
    } -> std::convertible_to<bool>;
};

}  // namespace prime_slam
