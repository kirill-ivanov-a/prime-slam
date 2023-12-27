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

#include "prime_slam/observation/detection/line/lsd.h"

namespace prime_slam::observation {

std::vector<OpenCVKeyline> observation::LSD::Detect(
    const cv::Mat& rgb_image) const {
  std::vector<cv::line_descriptor::KeyLine> keylines;
  lsd_->detect(rgb_image, keylines, scale_, octaves_number_);

  std::vector<OpenCVKeyline> result;
  result.reserve(keylines.size());
  std::transform(keylines.begin(), keylines.end(), std::back_inserter(result),
                 [](auto&& keyline) {
                   return OpenCVKeyline{keyline, 0};
                 });

  return result;
}

LSD::LSD(int scale, int octaves_number)
    : scale_(scale),
      octaves_number_(octaves_number),
      lsd_(cv::line_descriptor::LSDDetector::createLSDDetector()) {}

}  // namespace prime_slam::observation
