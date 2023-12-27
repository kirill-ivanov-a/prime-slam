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
#include <opencv2/line_descriptor.hpp>

#include "prime_slam/observation/keyobject_concept.h"

namespace prime_slam::observation {

/**
 * @brief Class representing key line for OpenCV algorithms
 */
class OpenCVKeyline final {
 public:
  using ObservationType = LineObservationTag;
  using CooridnatesType =
      Eigen::Vector<double,
                    ObservationTraits<ObservationType>::KeyobjectDimension>;

 public:
  /**
   * @brief Creates default key line
   */
  OpenCVKeyline() = default;

  /**
   * @brief Creates key line
   * @param keyline OpenCV internal key line
   * @param uncertainty Uncertainty of key line
   */
  OpenCVKeyline(const cv::line_descriptor::KeyLine& keyline, double uncertainty)
      : keyline_(keyline), uncertainty_(uncertainty) {}

  /**
   * @brief Returns coordinates of key line in image
   * @return key line coordinates
   */
  [[nodiscard]] CooridnatesType GetCoordinates() const noexcept {
    return Eigen::Vector4d{keyline_.startPointX, keyline_.startPointY,
                           keyline_.endPointX, keyline_.endPointY};
  }

  /**
   * @brief Returns uncertainty of key line
   * @return key line uncertainty
   */
  [[nodiscard]] double GetUncertainty() const noexcept { return uncertainty_; }

  /**
   * @brief Casts object to OpenCV internal key line
   * @return OpenCV internal key line object
   */
  explicit operator cv::line_descriptor::KeyLine() const noexcept {
    return keyline_;
  }

 private:
  const cv::line_descriptor::KeyLine keyline_;
  const double uncertainty_{0.0};
};

}  // namespace prime_slam::observation
