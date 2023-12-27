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

#include <Eigen/Geometry>

#include "prime_slam/data/datasets/data_format.h"

namespace prime_slam {

/**
 * @brief Default parameters for datasets
 * @tparam Format Concrete data format
 */
template <DataFormat Format>
class DefaultParameters;

/**
 * @brief Default parameters for ICL NUIM dataset
 */
template <>
class DefaultParameters<DataFormat::ICL> {
 public:
  /**
   * @brief Returns default camera intrinsics
   * @return intrinsics
   */
  static Eigen::Projective3d GetIntrinsics() {
    Eigen::Matrix4d intrinsics_matrix;
    intrinsics_matrix << 481.20, 0, 319.50, 0, 0, -480.00, 239.50, 0, 0, 0, 1,
        0, 0, 0, 0, 1;
    return Eigen::Projective3d{intrinsics_matrix};
  }

  /**
   * @brief Returns default extension of rgb file
   * @return file extension
   */
  static std::string GetRGBExtension() { return ".png"; }

  /**
   * @brief Returns default extension of depth file
   * @return file extension
   */
  static std::string GetDepthExtension() { return ".depth"; }

  /**
   * @brief Returns default extension of camera parameters file
   * @return file extension
   */
  static std::string GetCameraParametersExtension() { return ".txt"; }
};

/**
 * @brief Default parameters for ICL NUIM (TUM format) dataset
 */
template <>
class DefaultParameters<DataFormat::ICL_TUM> {
 public:
  /**
   * @brief Returns default camera intrinsics
   * @return intrinsics
   */
  static Eigen::Projective3d GetIntrinsics() {
    return DefaultParameters<DataFormat::ICL>::GetIntrinsics();
  }

  /**
   * @brief Returns default depth map scaling factor
   * @return Depth map scaling factor
   */
  static double GetDepthFactor() { return 5000.0; }

  /**
   * @brief Returns default name of directory with rgb images
   * @return Directory name
   */
  static std::string GetRGBDirectoryName() { return "rgb"; }

  /**
   * @brief Returns default name of directory with depthmaps
   * @return Directory name
   */
  static std::string GetDepthDirectoryName() { return "depth"; }

  /**
   * @brief Returns default name of file with GT poses
   * @return Filename
   */
  static std::string GetGTPosesFilename() { return "groundtruth.txt"; }
};

/**
 * @brief Default parameters for TUM RGBD dataset
 */
template <>
class DefaultParameters<DataFormat::TUM> {
 public:
  /**
   * @brief Returns default depth map scaling factor
   * @return Depth map scaling factor
   */
  static double GetDepthFactor() { return 5000.0; }

  /**
   * @brief Returns default name of directory with rgb images
   * @return Directory name
   */
  static std::string GetRGBDirectoryName() { return "rgb"; }

  /**
   * @brief Returns default name of directory with depth maps
   * @return Directory name
   */
  static std::string GetDepthDirectoryName() { return "depth"; }

  /**
   * @brief Returns default name of file with GT poses
   * @return Filename
   */
  static std::string GetGTPosesFilename() { return "groundtruth.txt"; }

  /**
   * @brief Returns default camera intrinsics
   * @return intrinsics
   */
  static Eigen::Projective3d GetIntrinsics() {
    Eigen::Matrix4d intrinsics_matrix;
    intrinsics_matrix << 525.0, 0, 319.5, 0, 0, 525.0, 239.5, 0, 0, 0, 1, 0, 0,
        0, 0, 1;
    return Eigen::Projective3d{intrinsics_matrix};
  }
};

}  // namespace prime_slam
