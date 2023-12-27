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

#include "prime_slam/metrics.h"

namespace prime_slam::metrics {

double AngularRotationError(const Eigen::Projective3d& first_pose,
                            const Eigen::Projective3d& second_pose) {
  auto&& first_rotation = first_pose.rotation();
  auto&& second_rotation = second_pose.rotation();
  auto cos = ((first_rotation * second_rotation.transpose()).trace() - 1) / 2;
  cos = std::clamp(cos, -1.0, 1.0);

  return std::abs(std::acos(cos)) * (std::numbers::pi_v<double> / 180.0);
}

double AngularTranslationError(const Eigen::Projective3d& first_pose,
                               const Eigen::Projective3d& second_pose) {
  auto&& first_translation = first_pose.translation();
  auto&& second_translation = second_pose.translation();
  auto norm_product = first_translation.norm() * second_translation.norm();
  if (norm_product == 0) {
    return 0;
  }
  auto cos = first_translation.dot(second_translation) / norm_product;
  cos = std::clamp(cos, -1.0, 1.0);

  return std::acos(cos) * (std::numbers::pi_v<double> / 180.0);
}

double AbsoluteTranslationError(const Eigen::Projective3d& first_pose,
                                const Eigen::Projective3d& second_pose) {
  auto delta_pose = first_pose * second_pose.inverse();

  return delta_pose.translation().norm();
}

}  // namespace prime_slam::metrics
