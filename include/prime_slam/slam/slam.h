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

#include <memory>
#include <utility>
#include <vector>

#include "prime_slam/slam/backend/backend_concept.h"
#include "prime_slam/slam/frontend/frontend_concept.h"

namespace prime_slam {

/**
 * @brief SLAM pipeline, including frontend and backend
 * @tparam FrontendType A specific frontend that satisfies the Frontend concept
 * @tparam BackendType A specific backend that satisfies the Backend concept
 */
template <frontend::Frontend FrontendType, backend::Backend BackendType>
class SLAM {
 public:
  /**
   * @brief Constructs a SLAM object from frontend and backend
   * @param frontend
   * @param backend
   */
  SLAM(FrontendType frontend, BackendType backend)
      : frontend_(std::move(frontend)), backend_(std::move(backend)) {}

  /**
   * @brief Processes a new frame
   * @param sensor_data RGBD image
   */
  void ProcessSensorData(sensor::RGBDImage sensor_data) {
    auto inserted = frontend_.TryInsertSensorData(std::move(sensor_data));
    if (inserted && frontend_.OptimizationRequired()) {
      auto&& graph = frontend_.GetGraph();
      auto backend_result = backend_.Optimize(graph);
      frontend_.UpdateGraph(std::move(backend_result));
    }
  }

  /**
   * @brief Returns the current map
   * @return Current map
   */
  const auto& GetMap() const& noexcept { return frontend_.GetMap(); }

  /**
   * @brief Returns the trajectory of frames
   * @return Absolute poses (as Eigen::Projective3d) for each keyframe
   */
  [[nodiscard]] std::vector<Eigen::Projective3d> GetTrajectory()
      const noexcept {
    return frontend_.GetTrajectory();
  }

 private:
  FrontendType frontend_;
  BackendType backend_;
};

}  // namespace prime_slam
