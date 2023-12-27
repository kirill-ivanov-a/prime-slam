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

#include "prime_slam/observation/detection/point/detail/opencv_point_detector.h"

#include <utility>

#include <opencv2/imgproc.hpp>

#include "prime_slam/observation/opencv_keypoint.h"

namespace prime_slam::observation::detail {

OpenCVPointDetector::OpenCVPointDetector(cv::Ptr<cv::FeatureDetector> detector)
    : detector_(std::move(detector)) {}

std::vector<OpenCVKeypoint> OpenCVPointDetector::Detect(
    const cv::Mat& rgb_image) const {
  cv::Mat gray;
  std::vector<cv::KeyPoint> keypoints;
  cv::cvtColor(rgb_image, gray, cv::COLOR_RGB2GRAY);
  detector_->detect(gray, keypoints);
  std::vector<OpenCVKeypoint> result;
  result.reserve(keypoints.size());
  std::transform(keypoints.begin(), keypoints.end(), std::back_inserter(result),
                 [](auto&& keypoint) {
                   return OpenCVKeypoint{keypoint, 0};
                 });

  return result;
}

}  // namespace prime_slam::observation::detail
