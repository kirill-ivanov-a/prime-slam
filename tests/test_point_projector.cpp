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

#include <gtest/gtest.h>
#include <prime_slam/prime_slam.h>

#include "common.h"

namespace prime_slam {

using prime_slam::observation::PointObservationTag;
using prime_slam::projection::DefaultProjector;
using Projector = prime_slam::projection::DefaultProjector<PointObservationTag>;

TEST(PointTransformation, Identity) {
  Projector projector;
  constexpr const auto kPointsNumber = 3;
  constexpr const auto kTolerance = 1e-10;
  std::vector<Eigen::Vector3d> points;
  points.reserve(kPointsNumber);
  for (auto i = 0; i != kPointsNumber; ++i) {
    points.emplace_back(Eigen::Vector3d::Random());
  }
  auto transformation = Eigen::Projective3d::Identity();
  auto transformed_points = projector.Transform(points, transformation);
  double max_difference = 0;
  for (auto i = 0; i != kPointsNumber; ++i) {
    double cur_max_diff =
        (points[i] - transformed_points[i]).cwiseAbs().maxCoeff();
    max_difference = std::max(max_difference, cur_max_diff);
  }

  EXPECT_LE(max_difference, kTolerance);
}

TEST(PointTransformation, Empty) {
  Projector projector;
  std::vector<Eigen::Vector3d> points;
  auto transformation = Eigen::Projective3d::Identity();
  auto transformed_points = projector.Transform(points, transformation);

  EXPECT_EQ(transformed_points.size(), 0);
}

TEST(PointTransformation, BackTransform) {
  Projector projector;
  constexpr const auto kPointsNumber = 3;
  constexpr const auto kTolerance = 1e-10;
  constexpr const auto kMaxAngle = 0.5;

  std::vector<Eigen::Vector3d> points;
  points.reserve(kPointsNumber);
  for (auto i = 0; i != kPointsNumber; ++i) {
    points.emplace_back(Eigen::Vector3d::Random());
  }

  auto transformation = GenerateRandomTransformation(kMaxAngle);
  auto transformed_points = projector.Transform(points, transformation);
  auto back_transformed_points =
      projector.Transform(transformed_points, transformation.inverse());

  double max_difference = 0;
  for (auto i = 0; i != kPointsNumber; ++i) {
    double cur_max_diff =
        (points[i] - back_transformed_points[i]).cwiseAbs().maxCoeff();
    max_difference = std::max(max_difference, cur_max_diff);
  }

  EXPECT_LE(max_difference, kTolerance);
}

TEST(PointProjection, Project) {
  Projector projector;
  constexpr const auto kPointsNumber = 3;
  constexpr const auto kTolerance = 1e-3;
  Eigen::Matrix4d intrinsics_matrix;
  intrinsics_matrix << 481.20, 0, 319.50, 0, 0, -480.00, 239.50, 0, 0, 0, 1, 0,
      0, 0, 0, 1;
  Eigen::Projective3d intrinsics{intrinsics_matrix};
  auto extrinsics = Eigen::Projective3d::Identity();

  auto points_3d = std::vector<Eigen::Vector3d>{
      {0, 1, 2}, {12, 4, 16}, {0, 0, 1}, {1, 1, 1}};

  auto projected_points = projector.Project(points_3d, intrinsics, extrinsics);

  auto expected_points = std::vector<Eigen::Vector2d>{
      {319.5, -0.5}, {680.4, 119.5}, {319.5, 239.5}, {800.7, -240.5}};

  double max_difference = 0;
  for (auto i = 0; i != kPointsNumber; ++i) {
    double cur_max_diff =
        (expected_points[i] - projected_points[i]).cwiseAbs().maxCoeff();
    max_difference = std::max(max_difference, cur_max_diff);
  }

  EXPECT_LE(max_difference, kTolerance);
}

TEST(PointProjection, BackProject) {
  Projector projector;
  constexpr const auto kPointsNumber = 3;
  constexpr const auto kTolerance = 1e-3;
  constexpr const auto kHeight = 16ul;
  constexpr const auto kWidth = 16ul;
  Eigen::Matrix4d intrinsics_matrix;
  intrinsics_matrix << 20, 0, 8, 0, 0, 20, 4, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Projective3d intrinsics{intrinsics_matrix};
  auto extrinsics = Eigen::Projective3d::Identity();

  auto points_2d =
      std::vector<Eigen::Vector2d>{{0, 1}, {0, 0}, {15, 3}, {14, 14}};
  auto depths = std::vector{0.5, 1.0, 7.0, 13.0};
  auto depth_map = CreateDepthMap(points_2d, depths, kHeight, kWidth);

  auto back_projected_points =
      projector.BackProject(points_2d, intrinsics, depth_map, extrinsics);

  auto expected_points = std::vector<Eigen::Vector3d>{{-0.2, -0.075, 0.5},
                                                      {-0.4, -0.2, 1.},
                                                      {2.45, -0.35, 7.},
                                                      {3.9, 6.5, 13.}};

  double max_difference = 0;
  for (auto i = 0; i != kPointsNumber; ++i) {
    double cur_max_diff =
        (expected_points[i] - back_projected_points[i]).cwiseAbs().maxCoeff();
    max_difference = std::max(max_difference, cur_max_diff);
  }

  EXPECT_LE(max_difference, kTolerance);
}

}  // namespace prime_slam
