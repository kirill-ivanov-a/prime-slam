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

#include <numbers>

#include <Eigen/Geometry>

namespace prime_slam::metrics {

/**
 * @brief Calculates angular rotation error between two poses.
 * @param first_pose First pose
 * @param second_pose Second pose
 * @return angle between rotations of two poses
 */
double AngularRotationError(const Eigen::Projective3d& first_pose,
                            const Eigen::Projective3d& second_pose);

/**
 * @brief Calculates angular translation error
 * @param first_pose First pose
 * @param second_pose Second pose
 * @return Angle (deg) between translation vectors of twgo poses
 */
double AngularTranslationError(const Eigen::Projective3d& first_pose,
                               const Eigen::Projective3d& second_pose);

/**
 * @brief Calculates absolute translation error between two poses.
 * @param first_pose First pose
 * @param second_pose Second pose
 * @return translation vector norm after applying reverse second_pose
 * transformation
 */
double AbsoluteTranslationError(const Eigen::Projective3d& first_pose,
                                const Eigen::Projective3d& second_pose);

}  // namespace prime_slam::metrics
