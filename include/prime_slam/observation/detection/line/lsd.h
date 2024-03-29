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

#include <opencv2/imgproc.hpp>
#include <opencv2/line_descriptor.hpp>

#include "prime_slam/observation/detection/detector_concept.h"
#include "prime_slam/observation/opencv_keyline.h"

namespace prime_slam::observation {

class LSD final {
 public:
  LSD(int scale = 2, int octaves_number = 1);

  std::vector<OpenCVKeyline> Detect(const cv::Mat& rgb_image) const;

 private:
  int scale_;
  int octaves_number_;
  cv::Ptr<cv::line_descriptor::LSDDetector> lsd_;
};

}  // namespace prime_slam::observation
