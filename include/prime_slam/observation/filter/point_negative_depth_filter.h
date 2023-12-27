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

#include <vector>
#include <utility>

#include "prime_slam/observation/filter/keyobject_filter_concept.h"

namespace prime_slam::observation {

/**
 * @brief Filters key points with negative depth
 * @tparam KeyobjectType Type of keypoint
 */
template <Keyobject KeyobjectType>
class PointNegativeDepthFilter {
 public:
  /**
   * @brief Returns keyobject with positive depth
   * @param keyobjects Key points to filter
   * @param sensor_data Corresponding RGBD image
   * @return Filtered key points
   */
  std::vector<KeyobjectType> Apply(std::vector<KeyobjectType> keyobjects,
                                   const sensor::RGBDImage& sensor_data) const;
};

// IMPLEMENTATION

template <Keyobject KeyobjectType>
inline std::vector<KeyobjectType>
PointNegativeDepthFilter<KeyobjectType>::Apply(
    std::vector<KeyobjectType> keyobjects,
    const sensor::RGBDImage& sensor_data) const {
  std::vector<KeyobjectType> filtered_keyobjects;
  filtered_keyobjects.reserve(keyobjects.size());

  auto&& depth_map = sensor_data.GetDepth();
  for (auto&& keyobject : keyobjects) {
    auto coords = keyobject.GetCoordinates();
    auto x = static_cast<int>(coords.x());
    auto y = static_cast<int>(coords.y());
    auto depth = depth_map(y, x);
    auto selected = depth > 0;
    if (selected) {
      filtered_keyobjects.push_back(std::move(keyobject));
    }
  }

  return filtered_keyobjects;
}

}  // namespace prime_slam::observation
