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

#include <Eigen/Core>
#include <opencv2/imgproc.hpp>

#include "prime_slam/observation/keyobject_concept.h"
#include "prime_slam/observation/traits/observation_tag.h"

namespace prime_slam::observation {

/**
 * @brief Class representing key point for OpenCV algorithms
 */
class OpenCVKeypoint final {
 public:
  using ObservationType = PointObservationTag;
  using CooridnatesType =
      Eigen::Vector<double,
                    ObservationTraits<ObservationType>::KeyobjectDimension>;

 public:
  /**
   * @brief Creates default key point
   */
  OpenCVKeypoint() = default;

  /**
   * @brief Creates key point
   * @param keypoint OpenCV internal key point
   * @param uncertainty Uncertainty of key point
   */
  OpenCVKeypoint(const cv::KeyPoint& keypoint, double uncertainty)
      : keypoint_(keypoint), uncertainty_(uncertainty) {}

  /**
   * @brief Returns coordinates of key point in image
   * @return key point coordinates
   */
  [[nodiscard]] CooridnatesType GetCoordinates() const noexcept {
    return Eigen::Vector2d{static_cast<double>(keypoint_.pt.x),
                           static_cast<double>(keypoint_.pt.y)};
  }

  /**
   * @brief Returns uncertainty of key point
   * @return key point uncertainty
   */
  [[nodiscard]] double GetUncertainty() const noexcept { return uncertainty_; }

  /**
   * @brief Casts object to OpenCV internal key point
   * @return OpenCV internal key point object
   */
  explicit operator cv::KeyPoint() const noexcept { return keypoint_; }

 private:
  cv::KeyPoint keypoint_;
  double uncertainty_{0.0};
};

}  // namespace prime_slam::observation
