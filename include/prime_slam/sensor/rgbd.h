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

#include <iostream>
#include <utility>

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

namespace prime_slam::sensor {

/**
 * @brief Class representing RGBD image
 */
class RGBDImage final {
 public:
  /**
   * @brief Default constructor
   */
  RGBDImage() {}

  /**
   * @brief Creates RGBD image from RGD image and depthmap
   * @param rgb_image RGB Image
   * @param depth_map Depthmap
   */
  RGBDImage(cv::Mat rgb_image, Eigen::MatrixXd depth_map)
      : rgb_image_(std::move(rgb_image)),
        depth_map_(std::move(depth_map)),
        height_(depth_map_.rows()),
        width_(depth_map_.cols()) {}

  /**
   * @brief Returns RGB image
   * @return RGB image
   */
  const cv::Mat& GetRGB() const& noexcept { return rgb_image_; }

  /**
   * @brief Returns depthmap
   * @return depthmap
   */
  const Eigen::MatrixXd& GetDepth() const& noexcept { return depth_map_; }

  /**
   * @brief Returns image width
   * @return image width
   */
  size_t GetWidth() const noexcept { return width_; }

  /**
   * @brief Returns image height
   * @return image height
   */
  size_t GetHeight() const noexcept { return height_; }

 private:
  const cv::Mat rgb_image_;
  const Eigen::MatrixXd depth_map_;
  const size_t height_{0};
  const size_t width_{0};
};

}  // namespace prime_slam::sensor
