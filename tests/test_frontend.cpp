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

namespace ps = prime_slam;

TEST(Frontend, ORB) {
  constexpr const auto kTolerance = 0.15;
  auto dataset = CreateTestDataset();
  auto&& gt_poses = dataset.GetGroundTruthPoses();
  auto frontend =
      ps::CreateDefaultORBFrontend(dataset.GetIntrinsics(), gt_poses.front());
  for (auto&& data : dataset) {
    frontend.TryInsertSensorData(std::move(data));
  }
  auto est_poses = frontend.GetTrajectory();
  auto max_ate = 0.0;
  for (auto&& [gt_pose, est_pose] : boost::combine(gt_poses, est_poses)) {
    auto ate = ps::metrics::AbsoluteTranslationError(est_pose, gt_pose);
    max_ate = std::max(max_ate, ate);
  }
  EXPECT_NEAR(max_ate, 0.0, kTolerance);
}

}  // namespace prime_slam
