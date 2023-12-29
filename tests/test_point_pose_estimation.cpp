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

using observation::PointObservationTag;
using Estimator = tracking::RGBDPoseEstimator<PointObservationTag>;
using Projector = projection::DefaultProjector<PointObservationTag>;

TEST(PointPoseEstimation, CorrectEstimation) {
  constexpr const auto kPointsNumber = 200;
  constexpr const auto kErrorTolerance = 1e-3;
  constexpr const auto kFirstFrameIndex = 0;
  constexpr const auto kSecondFrameIndex = 7;

  auto dataset = CreateTestDataset();
  auto&& gt_poses = dataset.GetGroundTruthPoses();
  auto first_frame = dataset[kFirstFrameIndex];
  auto second_frame = dataset[kSecondFrameIndex];
  auto&& first_frame_gt_pose = gt_poses[kFirstFrameIndex];
  auto&& second_frame_gt_pose = gt_poses[kSecondFrameIndex];
  auto&& gt_first_to_second_relative_pose =
      second_frame_gt_pose * first_frame_gt_pose.inverse();

  auto estimator = Estimator{};
  auto projector = Projector{};
  auto image_height = first_frame.GetHeight();
  auto image_width = first_frame.GetWidth();

  auto first_frame_points_2d = GenerateRandom2DPoints(
      kPointsNumber, static_cast<double>(image_width - 1),
      static_cast<double>(image_height - 1));
  auto first_frame_points_3d =
      projector.BackProject(first_frame_points_2d, dataset.GetIntrinsics(),
                            first_frame.GetDepth(), first_frame_gt_pose);
  auto second_frame_points_3d = projector.Transform(
      first_frame_points_3d, gt_first_to_second_relative_pose);
  auto second_frame_points_2d =
      projector.Project(second_frame_points_3d, dataset.GetIntrinsics(),
                        Eigen::Projective3d::Identity());

  std::vector<observation::Match> matches;
  matches.reserve(kPointsNumber);
  for (auto i = 0ul; i != kPointsNumber; ++i) {
    if (IsInImage(second_frame_points_2d[i], image_width, image_height)) {
      matches.push_back({i, i});
    }
  }

  auto estimated_pose =
      estimator.EstimatePose(second_frame_points_2d, dataset.GetIntrinsics(),
                             first_frame_points_3d, matches);

  auto angular_rotation_error = metrics::AngularRotationError(
      estimated_pose, gt_first_to_second_relative_pose);
  auto angular_translation_error = metrics::AngularTranslationError(
      estimated_pose, gt_first_to_second_relative_pose);
  auto absolute_translation_error = metrics::AbsoluteTranslationError(
      estimated_pose, gt_first_to_second_relative_pose);
  std::cout << gt_first_to_second_relative_pose.matrix() << '\n';
  std::cout << estimated_pose.matrix() << '\n';
  EXPECT_NEAR(angular_rotation_error, 0.0, kErrorTolerance);
  EXPECT_NEAR(angular_translation_error, 0.0, kErrorTolerance);
  EXPECT_NEAR(absolute_translation_error, 0.0, kErrorTolerance);
}

}  // namespace prime_slam
