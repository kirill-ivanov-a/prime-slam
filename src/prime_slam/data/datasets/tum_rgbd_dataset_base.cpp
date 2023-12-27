
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

#include "prime_slam/data/datasets/tum_rgbd_dataset_base.h"

#include "io_utils.h"

namespace prime_slam {

prime_slam::TUMRGBDDatasetBase::TUMRGBDDatasetBase(
    const fs::path& data_path, const std::string& rgb_directory_name,
    const std::string& depth_directory_name,
    const std::string& gt_poses_filename, Eigen::Projective3d intrinsics,
    double depth_factor)
    : rgb_base_path_(data_path / rgb_directory_name),
      depth_base_path_(data_path / depth_directory_name),
      gt_poses_path_(data_path / gt_poses_filename),
      rgb_images_paths_(GetDirectoryFiles(rgb_base_path_)),
      depths_paths_(GetDirectoryFiles(depth_base_path_)),
      gt_poses_(ReadPoses(gt_poses_path_)),
      intrinsics_(std::move(intrinsics)),
      depth_factor_(depth_factor) {}

}  // namespace prime_slam