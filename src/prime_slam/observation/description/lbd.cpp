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

#include "prime_slam/observation/description/line/lbd.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include "extract_cv_descriptors.h"

namespace prime_slam::observation {

LBD::LBD()
    : descriptor_(
          cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor()) {}

std::vector<Eigen::VectorXd> observation::LBD::Descript(
    const std::vector<OpenCVKeyline>& observations,
    const cv::Mat& rgb_image) const {
  std::vector<cv::line_descriptor::KeyLine> keylines;
  keylines.reserve(observations.size());
  std::transform(
      observations.begin(), observations.end(), std::back_inserter(keylines),
      [](auto&& observation) {
        return static_cast<cv::line_descriptor::KeyLine>(observation);
      });
  cv::Mat gray;
  cv::cvtColor(rgb_image, gray, cv::COLOR_RGB2GRAY);
  cv::Mat descriptors;
  descriptor_->compute(gray, keylines, descriptors);

  return ExtractCVDescriptors(descriptors);
}

}  // namespace prime_slam::observation
