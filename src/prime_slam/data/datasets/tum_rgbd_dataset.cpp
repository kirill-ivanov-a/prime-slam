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

#include "prime_slam/data/datasets/tum_rgbd_dataset.h"

#include <filesystem>

#include <Eigen/Eigen>
#include <fmt/core.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

namespace prime_slam {

std::vector<double> ExtractTimestamps(
    const std::vector<fs::directory_entry>& files) {
  std::vector<double> timestamps;
  timestamps.reserve(files.size());
  std::transform(
      files.begin(), files.end(), std::back_inserter(timestamps),
      [](const fs::directory_entry& file) {
        try {
          return std::stod(file.path().stem());
        } catch (const std::invalid_argument&) {
          throw std::invalid_argument{fmt::format(
              "Invalid file name: {}. Expected convertible to double timestamp",
              file.path().string())};
        }
      });

  return timestamps;
}

TUMRGBDDataset::TUMRGBDDataset(const fs::path& data_path,
                               const std::string& rgb_directory_name,
                               const std::string& depth_directory_name,
                               const std::string& gt_poses_filename,
                               Eigen::Projective3d intrinsics,
                               double depth_factor)
    : TUMRGBDDatasetBase(data_path, rgb_directory_name, depth_directory_name,
                         gt_poses_filename, std::move(intrinsics),
                         depth_factor),
      rgb_depth_associations_(
          CreateRGBToDepthAssociation(rgb_images_paths_, depths_paths_)) {}

std::vector<TUMRGBDDataset::RGBToDepthAssociation>
TUMRGBDDataset::CreateRGBToDepthAssociation(
    const std::vector<fs::directory_entry>& rgb_images_paths,
    const std::vector<fs::directory_entry>& depths_paths) {
  auto timestamps_rgb = ExtractTimestamps(rgb_images_paths);
  auto timestamps_depth = ExtractTimestamps(depths_paths);

  Eigen::Map<Eigen::VectorXd> timestamps_rgb_eigen(timestamps_rgb.data(),
                                                   timestamps_rgb.size());
  Eigen::Map<Eigen::VectorXd> timestamps_depth_eigen(timestamps_depth.data(),
                                                     timestamps_depth.size());
  std::vector<size_t> min_distances_indices(timestamps_rgb_eigen.size());

  for (auto i = 0ul; i != timestamps_rgb_eigen.size(); ++i) {
    double min_distance = std::numeric_limits<double>::max();

    for (auto j = 0ul; j != timestamps_depth_eigen.size(); ++j) {
      double distance =
          std::abs(timestamps_rgb_eigen(i) - timestamps_depth_eigen(j));

      if (distance < min_distance) {
        min_distances_indices[i] = j;
      }
    }
  }
  std::vector<RGBToDepthAssociation> result;
  result.reserve(min_distances_indices.size());
  for (auto i = 0ul; i != min_distances_indices.size(); ++i) {
    result.push_back({i, min_distances_indices[i]});
  }

  return result;
}

sensor::RGBDImage TUMRGBDDataset::operator[](size_t index) const {
  auto rgb_depth_association = rgb_depth_associations_[index];
  auto rgb_img =
      cv::imread(rgb_images_paths_[rgb_depth_association.rgb_index].path());
  auto cv_depth =
      cv::imread(depths_paths_[rgb_depth_association.depth_index].path(),
                 cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
  cv_depth.convertTo(cv_depth, CV_64F);
  Eigen::MatrixXd depth;
  cv::cv2eigen(cv_depth, depth);
  depth /= depth_factor_;
  return sensor::RGBDImage{std::move(rgb_img), std::move(depth)};
}

}  // namespace prime_slam
