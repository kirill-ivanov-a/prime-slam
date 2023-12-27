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

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

#include "prime_slam/data/datasets/data_format.h"
#include "prime_slam/data/datasets/default_parameters.h"
#include "prime_slam/data/datasets/tum_rgbd_dataset_base.h"

namespace prime_slam {

/**
 * @brief Dataset [for ICL
 * NUIM (TUM format)](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)
 */
class ICLNUIMTUMFormatDataset final : public TUMRGBDDatasetBase {
  using Parameters = DefaultParameters<DataFormat::ICL_TUM>;

  /**
   * @brief Creates dataset object
   * @param data_path Path to the directory with data in [TUM
   * format](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)
   * @param rgb_directory_name Name of directory with rgb images
   * @param depth_directory_name Name of directory with depthmaps
   * @param gt_poses_filename Name of file with GT poses
   * @param intrinsics Camera intrinsics
   * @param depth_factor Depthmap scale factor
   */
 public:
  ICLNUIMTUMFormatDataset(
      const fs::path& data_path,
      const std::string& rgb_directory_name = Parameters::GetRGBDirectoryName(),
      const std::string& depth_directory_name =
          Parameters::GetDepthDirectoryName(),
      const std::string& gt_poses_filename = Parameters::GetGTPosesFilename(),
      Eigen::Projective3d intrinsics = Parameters::GetIntrinsics(),
      double depth_factor = Parameters::GetDepthFactor());

  /**
   * @brief Returns the RGBD Image at the corresponding index
   * @param index Index of dataset RGBD Image
   * @return RGBD Image
   */
  sensor::RGBDImage operator[](size_t index) const override;
};

}  // namespace prime_slam
