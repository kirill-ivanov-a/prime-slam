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

#include "prime_slam/slam/frame/keyframe_selection/keyframe_selector_concept.h"
#include "prime_slam/slam/mapping/map_concept.h"
#include "prime_slam/slam/mapping/mapper.h"
#include "prime_slam/slam/tracking/tracker.h"
#include "prime_slam/slam/tracking/tracking_result.h"

namespace prime_slam::frontend {

/**
 * @brief Frontend concept
 * @tparam FrontendImpl Concrete frontend
 */
template <typename FrontendImpl>
concept Frontend = std::movable<FrontendImpl> &&
    requires(FrontendImpl frontend) {
  // Takes a sensor data and returns true if data is inserted
  {
    frontend.TryInsertSensorData(std::declval<sensor::RGBDImage>())
    } -> std::convertible_to<bool>;
  // Returns true if factor graph optimization is required
  { frontend.OptimizationRequired() } -> std::convertible_to<bool>;
  // Returns the keyframe trajectory
  {
    frontend.GetTrajectory()
    } -> std::convertible_to<std::vector<Eigen::Projective3d>>;
  // Returns the current representation of the factor graph
  { frontend.GetGraph() } -> std::convertible_to<factor_graph::FactorGraph>;
  // Updates the graph with backend-optimized data
  {frontend.UpdateGraph(std::declval<backend::BackendResult>())};
  // GetMap should return implementation of map concept
  requires mapping::Map<std::decay_t<decltype(frontend.GetMap())>>;
};

}  // namespace prime_slam::frontend
