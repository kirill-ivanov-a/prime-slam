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

#include <Eigen/Core>

#include "data_paths.h"
#include "prime_slam/data/datasets/data_format.h"

namespace prime_slam {

namespace fs = std::filesystem;

bool IsInImage(const Eigen::Vector2d& point, size_t image_width,
               size_t image_height) {
  auto x = point[0];
  auto y = point[1];
  return (x < static_cast<double>(image_width) && x >= 0 &&
          y < static_cast<double>(image_height) && y >= 0);
}

auto CreateTestDataset() {
  auto base_path = kTestDatasetPath;
  auto dataset_format = DataFormat::ICL_TUM;
  auto dataset = DatasetFactory::Create(dataset_format, base_path);

  return dataset;
}

std::vector<Eigen::Vector2d> GenerateRandom2DPoints(size_t points_number,
                                                    double x_scaler = 1,
                                                    double y_scaler = 1) {
  std::vector<Eigen::Vector2d> points;
  points.reserve(points_number);
  for (auto i = 0; i != points_number; ++i) {
    auto point = Eigen::Vector2d::Random().eval();
    point.array() += 1;
    point.array() /= 2;
    point(0) *= x_scaler;
    point(1) *= y_scaler;
    points.push_back(point);
  }

  return points;
}

Eigen::Matrix3d GenerateRotation(double alpha, double beta, double gamma) {
  Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitZ()))
          .toRotationMatrix();
  return rotation;
}

Eigen::MatrixXd CreateDepthMap(const std::vector<Eigen::Vector2d>& positions,
                               const std::vector<double>& depths, size_t height,
                               size_t width) {
  Eigen::MatrixXd depth_map(height, width);
  for (auto&& [position, depth] : boost::combine(positions, depths)) {
    auto x = static_cast<int>(position[0]);
    auto y = static_cast<int>(position[1]);
    depth_map(y, x) = depth;
  }
  return depth_map;
}

Eigen::Matrix3d GenerateRandomRotation(double max_angle) {
  Eigen::Vector3d angles = max_angle * Eigen::Vector3d::Random();
  return GenerateRotation(angles[0], angles[1], angles[2]);
}

Eigen::Projective3d GenerateRandomTransformation(double max_angle) {
  Eigen::Matrix3d rotation = GenerateRandomRotation(max_angle);
  Eigen::Vector3d translation = Eigen::Vector3d::Random();
  Eigen::Matrix4d transformation_matrix;
  transformation_matrix.setIdentity();
  transformation_matrix.block<3, 3>(0, 0) = rotation;
  transformation_matrix.block<3, 1>(0, 3) = translation;
  return Eigen::Projective3d{transformation_matrix};
}

}  // namespace prime_slam
