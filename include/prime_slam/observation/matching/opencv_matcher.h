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
#include <vector>

#include <Eigen/Core>
#include <opencv2/features2d.hpp>

#include "prime_slam/observation/matching/matcher_concept.h"

namespace prime_slam::observation {

/**
 * @brief Descriptor matcher based on OpenCV
 */
class OpenCVMatcher final {
 public:
  /**
   * @brief Creates default matched
   * @param matcher_type Desired matcher type
   * @param ratio_threshold Threshold for [ratio
   * test](https://docs.opencv.org/3.4/dc/dc3/tutorial_py_matcher.html)
   */
  OpenCVMatcher(cv::DescriptorMatcher::MatcherType matcher_type =
                    cv::DescriptorMatcher::BRUTEFORCE_HAMMING,
                double ratio_threshold = 0.7)
      : matcher_(cv::DescriptorMatcher::create(matcher_type)),
        ratio_threshold_(ratio_threshold) {}

  /**
   * @brief Matches two sets of descriptors
   * @param reference_descriptors Descriptors from reference frame
   * @param target_descriptors Descriptors from target frame
   * @return Matches between reference and target frame descriptors
   */
  std::vector<Match> MatchDescriptors(
      const std::vector<Eigen::VectorXd>& reference_descriptors,
      const std::vector<Eigen::VectorXd>& target_descriptors) const;

 private:
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  double ratio_threshold_;
};

}  // namespace prime_slam::observation
