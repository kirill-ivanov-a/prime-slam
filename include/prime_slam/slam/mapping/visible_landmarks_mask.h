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

#include <boost/range/combine.hpp>
#include <Eigen/Core>

#include "prime_slam/observation/traits/observation_tag.h"
#include "prime_slam/observation/traits/observation_traits.h"
#include "prime_slam/projection/line_projector.h"
#include "prime_slam/slam/frame/frame.h"

namespace prime_slam::mapping {

/**
 * @brief Class representing mask of visible from frame landmarks (no common
 * implementation)
 * @tparam Observation Type of observation
 */
template <std::derived_from<observation::ObservationTag> Observation>
class VisibleLandmarksMask;

/**
 * @brief Class representing mask of visible from frame point landmarks
 */
template <>
class VisibleLandmarksMask<observation::PointObservationTag> {
  using ObservationType = observation::PointObservationTag;
  using LandmarkCoordinates =
      Eigen::Vector<double,
                    observation::ObservationTraits<
                        observation::PointObservationTag>::LandmarkDimension>;

 public:
  /**
   * @brief Creates boolean mask of visible landmarks
   * @param landmark_positions Map Point Landmark positions
   * @param frame Desired frame
   * @return boolean mask of visible landmarks
   */
  static std::vector<bool> Create(
      const std::vector<LandmarkCoordinates>& landmark_positions,
      const Frame<ObservationType>& frame);
};

// IMPLEMENTATION

inline std::vector<bool>
VisibleLandmarksMask<observation::PointObservationTag>::Create(
    const std::vector<LandmarkCoordinates>& landmark_positions,
    const Frame<ObservationType>& frame) {
  auto projector = prime_slam::projection::DefaultProjector<ObservationType>{};
  auto landmark_positions_cam = projector.Transform(
      landmark_positions, frame.GetWorldToCameraTransform());
  auto projected_map =
      projector.Project(landmark_positions_cam, frame.GetIntrinsics(),
                        Eigen::Projective3d::Identity());

  auto&& sensor_data = frame.GetSensorData();
  auto height = sensor_data.GetHeight();
  auto width = sensor_data.GetWidth();

  std::vector<bool> mask;
  mask.reserve(landmark_positions.size());
  for (auto&& [projected_landmark, landmark_position_cam] :
       boost::combine(projected_map, landmark_positions_cam)) {
    auto x = projected_landmark(0);
    auto y = projected_landmark(1);
    auto depth = landmark_position_cam(2);
    mask.push_back((x >= 0) && (x < width) && (y >= 0) && (y < height) &&
                   (depth > 0));
  }

  return mask;
}

}  // namespace prime_slam::mapping
