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

#include "prime_slam/slam/frame/keyframe_selection/keyframe_selector_concept.h"

namespace prime_slam {

/**
 * @brief A selector that considers every nth frame to be a key frame
 * @tparam ObservationType Type of observation
 */
template <std::derived_from<ObservationTag> ObservationType>
class EveryNthKeyframeSelector {
 public:
  /**
   * @brief Constructs keyframe selector
   * @param n Step between keyframes
   */
  EveryNthKeyframeSelector(size_t n = 10) : n_(n) {}

  /**
   * @brief Determines whether a frame is a keyframe
   * @return True, if n frames have passed since the last key frame was selected
   */
  bool IsSelected(const Frame<ObservationType>&) {
    ++counter_;
    auto selected = (counter_ / n_) == 1;
    if (selected) {
      counter_ = 0;
    }

    return selected;
  }

 private:
  size_t n_;
  size_t counter_{0};
};

}  // namespace prime_slam
