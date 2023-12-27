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

#include "prime_slam/observation/description/point/detail/opencv_point_descriptor.h"

#include <utility>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include "extract_cv_descriptors.h"

namespace prime_slam::observation::detail {

OpenCVPointDescriptor::OpenCVPointDescriptor(
    cv::Ptr<cv::DescriptorExtractor> descriptor)
    : descriptor_(std::move(descriptor)) {}

std::vector<Eigen::VectorXd> OpenCVPointDescriptor::Descript(
    const std::vector<observation::OpenCVKeypoint>& observations,
    const cv::Mat& rgb_image) const {
  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(observations.size());
  std::transform(observations.begin(), observations.end(),
                 std::back_inserter(keypoints), [](auto&& observation) {
                   return static_cast<cv::KeyPoint>(observation);
                 });
  cv::Mat gray;
  cv::Mat descriptors;
  cv::cvtColor(rgb_image, gray, cv::COLOR_RGB2GRAY);
  descriptor_->compute(gray, keypoints, descriptors);

  return ExtractCVDescriptors(descriptors);
}

}  // namespace prime_slam::observation::detail
