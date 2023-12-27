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

#include "prime_slam/observation/matching/opencv_matcher.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>

#include "prime_slam/observation/matching/match.h"

namespace prime_slam::observation {

namespace {

cv::Mat ConvertToCV(const std::vector<Eigen::VectorXd>& descriptors) {
  assert(!descriptors.empty());
  auto rows = descriptors.size();
  auto cols = descriptors.front().size();
  cv::Mat descriptors_cv(rows, cols, CV_8U);
  for (auto i = 0; auto&& descriptor : descriptors) {
    auto&& row = descriptors_cv.row(i++);
    std::transform(descriptor.begin(), descriptor.end(), row.begin<uint8_t>(),
                   [](auto&& value) { return static_cast<uint8_t>(value); });
  }
  return descriptors_cv;
}

}  // namespace

std::vector<Match> OpenCVMatcher::MatchDescriptors(
    const std::vector<Eigen::VectorXd>& reference_descriptors,
    const std::vector<Eigen::VectorXd>& target_descriptors) const {
  if (reference_descriptors.empty() || target_descriptors.empty()) {
    return {};
  }
  std::vector<std::vector<cv::DMatch>> matches;
  auto reference_descriptors_cv = ConvertToCV(reference_descriptors);
  auto target_descriptors_cv = ConvertToCV(target_descriptors);
  matcher_->knnMatch(reference_descriptors_cv, target_descriptors_cv, matches,
                     2);
  std::vector<Match> good_matches;
  for (auto& match : matches) {
    if (match[0].distance < ratio_threshold_ * match[1].distance) {
      good_matches.push_back({static_cast<size_t>(match[0].queryIdx),
                              static_cast<size_t>(match[0].trainIdx)});
    }
  }
  return good_matches;
}

}  // namespace prime_slam::observation
