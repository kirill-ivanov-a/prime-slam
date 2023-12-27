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

#include <Eigen/Geometry>

#include "prime_slam/observation/matching/match.h"
#include "prime_slam/slam/frame/frame.h"
#include "prime_slam/slam/mapping/map.h"

namespace prime_slam::tracking {
using observation::ObservationTag;
using observation::ObservationTraits;

template <std::derived_from<ObservationTag> ObservationType>
using LandmarkCoordinatesType = Eigen::Vector<
    double, observation::ObservationTraits<ObservationType>::LandmarkDimension>;

template <std::derived_from<ObservationTag> ObservationType>
using KeyobjectCoordinatesType =
    Eigen::Vector<double, observation::ObservationTraits<
                              ObservationType>::KeyobjectDimension>;

/**
 * @brief PoseEstimator concept
 * @tparam PoseEstimatorImpl Concrete Pose Estimator type
 */
template <typename PoseEstimatorImpl, typename ObservationType>
concept PoseEstimator = std::derived_from<ObservationType, ObservationTag> &&
    std::default_initializable<PoseEstimatorImpl> &&
    std::movable<PoseEstimatorImpl> && requires(PoseEstimatorImpl estimator) {
  estimator;
  // estimates pose of a frame relative to a set of 3D objects based on its
  // 2D observations
  {
    estimator.EstimatePose(
        /*observations_2d=*/std::declval<
            std::vector<KeyobjectCoordinatesType<ObservationType>>>(),
        /*camera_intrinsics=*/std::declval<Eigen::Projective3d>(),
        /*landmarks_coordinates=*/
        std::declval<std::vector<LandmarkCoordinatesType<ObservationType>>>(),
        /*matches=*/std::declval<std::vector<observation::Match>>())
    } -> std::convertible_to<Eigen::Projective3d>;
};

}  // namespace prime_slam::tracking
