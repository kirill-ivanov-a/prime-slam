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

#include "prime_slam/data/datasets/data_format.h"
#include "prime_slam/data/datasets/default_parameters.h"
#include "prime_slam/data/datasets/rgbd_dataset_interface.h"

namespace prime_slam {

namespace fs = std::filesystem;

/**
 * @brief Dataset [for ICL
 * NUIM](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)
 */
class ICLNUIMDataset final : public IRGBDDataset {
  using Parameters = DefaultParameters<DataFormat::ICL>;

 public:
  /**
   * @brief Creates dataset object
   * @param data_path Path to the directory with data in [TUM
   * @param rgb_extension Default extension of rgb file
   * @param depth_extension Default extension of depth file
   * @param camera_parameters_extension Default extension of camera parameters
   * file
   * @param intrinsics Default camera intrinsics
   */
  ICLNUIMDataset(
      const fs::path& data_path,
      const std::string& rgb_extension = Parameters::GetRGBExtension(),
      const std::string& depth_extension = Parameters::GetDepthExtension(),
      const std::string& camera_parameters_extension =
          Parameters::GetCameraParametersExtension(),
      const Eigen::Projective3d& intrinsics = Parameters::GetIntrinsics());

  /**
   * @brief Returns the RGBD Image at the corresponding index
   * @param index Index of dataset RGBD Image
   * @return RGBD Image
   */
  sensor::RGBDImage operator[](size_t) const override;

  /**
   * @brief Returns size of the dataset
   * @return dataset size
   */
  [[nodiscard]] size_t Size() const noexcept override;

  /**
   * @brief Returns groundtruth poses for each frame (RGBD Image)
   * @return Groundtruth poses
   */
  [[nodiscard]] const std::vector<Eigen::Projective3d>& GetGroundTruthPoses()
      const& noexcept override;

  /**
   * @brief Returns corresponding camera intrinsics
   * @return Camera intrinsics
   */
  [[nodiscard]] const Eigen::Projective3d& GetIntrinsics()
      const& noexcept override;

 private:
  [[nodiscard]] std::vector<Eigen::Projective3d> CreateGroundTruthPoses() const;

 private:
  std::vector<std::filesystem::directory_entry> rgb_images_paths_;
  std::vector<std::filesystem::directory_entry> depths_paths_;
  std::vector<std::filesystem::directory_entry> camera_parameters_paths_;
  std::vector<Eigen::Projective3d> gt_poses_;
  Eigen::Projective3d intrinsics_;
};

}  // namespace prime_slam
