
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

#include "prime_slam/data/datasets/icl_nuim_tum_format_dataset.h"

namespace prime_slam {

ICLNUIMTUMFormatDataset::ICLNUIMTUMFormatDataset(
    const fs::path& data_path, const std::string& rgb_directory_name,
    const std::string& depth_directory_name,
    const std::string& gt_poses_filename, Eigen::Projective3d intrinsics,
    double depth_factor)
    : TUMRGBDDatasetBase(data_path, rgb_directory_name, depth_directory_name,
                         gt_poses_filename, intrinsics, depth_factor) {}

sensor::RGBDImage ICLNUIMTUMFormatDataset::operator[](size_t index) const {
  auto rgb_img = cv::imread(rgb_images_paths_[index].path());
  auto cv_depth = cv::imread(depths_paths_[index].path(),
                             cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
  cv_depth.convertTo(cv_depth, CV_64F);
  Eigen::MatrixXd depth;
  cv::cv2eigen(cv_depth, depth);
  depth /= depth_factor_;
  return sensor::RGBDImage{std::move(rgb_img), std::move(depth)};
}

}  // namespace prime_slam
