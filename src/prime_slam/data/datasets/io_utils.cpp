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

#include "io_utils.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <fmt/core.h>
#include <boost/algorithm/string.hpp>
#include <Eigen/Geometry>

namespace prime_slam {

namespace fs = std::filesystem;

std::vector<Eigen::Projective3d> ReadPoses(
    const std::filesystem::path& poses_path,
    const std::string& comment_symbol) {
  if (!fs::exists(poses_path)) {
    auto msg = fmt::format("File does not exist: {}", poses_path.string());
    throw fs::filesystem_error(msg, poses_path, std::error_code());
  }
  std::vector<Eigen::Projective3d> poses;
  std::ifstream file(poses_path);
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      boost::trim(line);
      if (!boost::algorithm::starts_with(line, comment_symbol)) {
        std::istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;

        iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        auto rotation = Eigen::Quaterniond{qw, qx, qy, qz}.matrix();
        auto translation = Eigen::Vector3d{tx, ty, tz};
        auto transformation_matrix = Eigen::Matrix4d::Identity().eval();
        transformation_matrix.block<3, 3>(0, 0) = rotation;
        transformation_matrix.block<3, 1>(0, 3) = translation;

        poses.push_back(Eigen::Projective3d{transformation_matrix}.inverse());
      }
    }
  } else {
    auto msg = fmt::format("Failed to open file: {}", poses_path.string());
    throw std::ifstream::failure(msg);
  }

  return poses;
}

std::vector<std::filesystem::directory_entry> GetDirectoryFiles(
    const fs::path& base_path) {
  if (!std::filesystem::is_directory(base_path)) {
    throw std::invalid_argument("The specified path is not a directory.");
  }
  std::vector<std::filesystem::directory_entry> files;

  for (auto&& entry : std::filesystem::directory_iterator(base_path)) {
    if (std::filesystem::is_regular_file(entry)) {
      files.push_back(entry);
    }
  }
  std::sort(files.begin(), files.end(),
            [](const std::filesystem::directory_entry& lhs,
               const std::filesystem::directory_entry& rhs) {
              return std::stod(lhs.path().stem().string()) <
                     std::stod(rhs.path().stem().string());
            });

  return files;
}

}  // namespace prime_slam
