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

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "prime_slam/observation/detection/point/detail/opencv_point_detector.h"
#include "prime_slam/observation/keyobject_concept.h"
#include "prime_slam/observation/traits/observation_tag.h"

namespace prime_slam::observation {

/**
 * @brief Class representing [ORB
 * detector](https://docs.opencv.org/4.x/d1/d89/tutorial_py_orb.html)
 */
class ORB final : public detail::OpenCVPointDetector {
 public:
  using ObservationType = PointObservationTag;

  ORB(size_t features_number = 1000)
      : OpenCVPointDetector(cv::ORB::create(features_number)) {}
};

}  // namespace prime_slam::observation
