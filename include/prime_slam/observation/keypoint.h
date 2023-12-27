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

#include <utility>

#include "prime_slam/observation/keyobject_concept.h"
#include "prime_slam/observation/traits/observation_tag.h"

namespace prime_slam::observation {

/**
 * @brief Class representing key point
 */
class Keypoint final {
 public:
  using ObservationType = PointObservationTag;
  using CooridnatesType =
      Eigen::Vector<double,
                    ObservationTraits<ObservationType>::KeyobjectDimension>;

 public:
  /**
   * @brief Creates default key point
   */
  Keypoint() = default;

  /**
   * @brief Creates key point
   * @param coordinates Coordinates of key point in image
   * @param uncertainty Uncertainty of key point
   */
  Keypoint(Eigen::Vector2d coordinates, double uncertainty = 1.0)
      : coordinates_(std::move(coordinates)), uncertainty_(uncertainty) {}

  /**
   * @brief Returns coordinates of key point in image
   * @return key point coordinates
   */
  [[nodiscard]] CooridnatesType GetCoordinates() const noexcept {
    return coordinates_;
  }

  /**
   * @brief Returns uncertainty of key point
   * @return key point uncertainty
   */
  [[nodiscard]] double GetUncertainty() const noexcept { return uncertainty_; }

 private:
  Eigen::Vector2d coordinates_;
  double uncertainty_{1.0};
};
}  // namespace prime_slam::observation
