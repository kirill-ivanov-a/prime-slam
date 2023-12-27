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

#include <filesystem>
#include <string>
#include <vector>

#include "prime_slam/data/datasets/rgbd_dataset_interface.h"

namespace prime_slam {

namespace fs = std::filesystem;

/**
 * @brief Base class for datasets in [TUM
 * format](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)
 */
class TUMRGBDDatasetBase : public IRGBDDataset {
 public:
  /**
   * @brief Initializes base dataset object
   * @param data_path Path to the directory with data in [TUM
   * format](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)
   * @param rgb_directory_name Name of directory with rgb images
   * @param depth_directory_name Name of directory with depthmaps
   * @param gt_poses_filename Name of file with GT poses
   * @param intrinsics Camera intrinsics
   * @param depth_factor Depthmap scale factor
   */
  TUMRGBDDatasetBase(const fs::path& data_path,
                     const std::string& rgb_directory_name,
                     const std::string& depth_directory_name,
                     const std::string& gt_poses_filename,
                     Eigen::Projective3d intrinsics, double depth_factor);

  /**
   * @brief Returns size of the dataset
   * @return dataset size
   */
  [[nodiscard]] size_t Size() const noexcept override {
    return rgb_images_paths_.size();
  }

  /**
   * @brief Returns groundtruth poses for each frame (RGBD Image)
   * @return Groundtruth poses
   */
  const std::vector<Eigen::Projective3d>& GetGroundTruthPoses()
      const& noexcept override {
    return gt_poses_;
  }

  /**
   * @brief Returns corresponding camera intrinsics
   * @return Camera intrinsics
   */
  const Eigen::Projective3d& GetIntrinsics() const& noexcept override {
    return intrinsics_;
  }

 protected:
  fs::path rgb_base_path_;
  std::string depth_base_path_;
  std::string gt_poses_path_;
  std::vector<std::filesystem::directory_entry> rgb_images_paths_;
  std::vector<std::filesystem::directory_entry> depths_paths_;
  std::vector<Eigen::Projective3d> gt_poses_;
  Eigen::Projective3d intrinsics_;
  double depth_factor_;
};

}  // namespace prime_slam
