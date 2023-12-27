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

#include <Eigen/Geometry>

namespace prime_slam::projection {

using observation::ObservationTag;
using observation::ObservationTraits;
using Intrinsics = Eigen::Projective3d;
using Extrinsics = Eigen::Projective3d;
using Transformation = Eigen::Projective3d;
using DepthMap = Eigen::MatrixXd;

template <std::derived_from<ObservationTag> ObservationType>
using LandmarkCoordinatesType =
    Eigen::Vector<double,
                  ObservationTraits<ObservationType>::LandmarkDimension>;

template <std::derived_from<ObservationTag> ObservationType>
using KeyobjectCoordinatesType =
    Eigen::Vector<double,
                  ObservationTraits<ObservationType>::KeyobjectDimension>;

/**
 * @brief Concept of projector
 * @tparam ProjectorImpl Concrete type of projector
 * @tparam ObservationType Type of observation to project
 */
template <typename ProjectorImpl, typename ObservationType>
concept Projector = std::default_initializable<ProjectorImpl> &&
    std::movable<ProjectorImpl> && requires(ProjectorImpl projector) {
  // Transforms given coordinates of observation
  {
    projector.Transform(
        std::declval<std::vector<LandmarkCoordinatesType<ObservationType>>>(),
        std::declval<Transformation>())
    } -> std::convertible_to<
        std::vector<LandmarkCoordinatesType<ObservationType>>>;
  // Projects given coordinates of observation
  {
    projector.Project(
        std::declval<std::vector<LandmarkCoordinatesType<ObservationType>>>(),
        std::declval<Intrinsics>(), std::declval<Extrinsics>())
    } -> std::convertible_to<
        std::vector<KeyobjectCoordinatesType<ObservationType>>>;
  // Back projects given coordinates of observation
  {
    projector.BackProject(
        std::declval<std::vector<KeyobjectCoordinatesType<ObservationType>>>(),
        std::declval<Intrinsics>(), std::declval<DepthMap>(),
        std::declval<Extrinsics>())
    } -> std::convertible_to<
        std::vector<LandmarkCoordinatesType<ObservationType>>>;
};

}  // namespace prime_slam::projection
