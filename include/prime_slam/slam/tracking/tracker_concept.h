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

#include "prime_slam/slam/frame/frame.h"
#include "prime_slam/slam/mapping/map.h"
#include "prime_slam/slam/mapping/map_concept.h"
#include "prime_slam/slam/tracking/tracking_result.h"

namespace prime_slam::tracking {

/**
 * @brief Tracker concept
 * @tparam TrackerImpl Concrete Tracker type
 * @tparam MapImpl Concrete Map Type
 */
template <typename TrackerImpl, typename MapImpl>
concept Tracker = std::movable<TrackerImpl> && mapping::Map<MapImpl> &&
    requires(TrackerImpl tracker, MapImpl map) {
  // Tracks frame changes relative to the map
  {
    tracker.TrackMap(std::declval<Frame<typename MapImpl::ObservationType>>(),
                     map)
    } -> std::convertible_to<TrackingResult>;
  // Tracks a new frame relative to the previous one
  {
    tracker.Track(std::declval<Frame<typename MapImpl::ObservationType>>(),
                  std::declval<Frame<typename MapImpl::ObservationType>>())
    } -> std::convertible_to<TrackingResult>;
};

}  // namespace prime_slam::tracking
