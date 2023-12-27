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

#include <Eigen/Core>
#include <opencv2/line_descriptor.hpp>

#include "prime_slam/observation/detection/detector_concept.h"
#include "prime_slam/observation/opencv_keyline.h"

namespace prime_slam::observation {

/**
 * @brief Class representing [LBD
 * descriptor](https://docs.opencv.org/4.x/dc/ddd/group__line__descriptor.html)
 */
class LBD final {
 public:
  /**
   * @brief Creates LBD descriptor
   */
  LBD();

  /**
   * @brief Returns descriptors for each key line
   * @param observations Detected key lines
   * @param rgb_image Corresponding RGB image
   * @return vector of descriptors
   */
  std::vector<Eigen::VectorXd> Descript(
      const std::vector<OpenCVKeyline>& observations,
      const cv::Mat& rgb_image) const;

 private:
  cv::Ptr<cv::line_descriptor::BinaryDescriptor> descriptor_;
};

}  // namespace prime_slam::observation
