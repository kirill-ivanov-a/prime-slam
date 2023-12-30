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

using MapType = ps::mapping::DefaultMap<ps::observation::PointObservationTag>;
using FrameType = ps::Frame<ps::observation::PointObservationTag>;
using TrackerType = ps::tracking::DefaultTracker<MapType>;
using MapperType = ps::mapping::DefaultMapper<MapType>;

TEST(Tracker, Track) {
  constexpr const auto kTolerance = 0.03;
  constexpr const auto kFirstFrameIndex = 0;
  constexpr const auto kSecondFrameIndex = 7;

  auto dataset = CreateTestDataset();
  auto&& gt_poses = dataset.GetGroundTruthPoses();
  auto gt_relative_pose =
      gt_poses[kSecondFrameIndex] * gt_poses[kFirstFrameIndex].inverse();
  auto observation_creator = ps::CreateDefaultORBObservationCreator();
  auto first_rgbd = dataset[kFirstFrameIndex];
  auto second_rgbd = dataset[kSecondFrameIndex];
  auto first_observation_data =
      observation_creator.Create(dataset[kFirstFrameIndex]);
  auto second_observation_data =
      observation_creator.Create(dataset[kSecondFrameIndex]);
  auto first_frame = FrameType{kFirstFrameIndex, first_observation_data,
                               std::move(first_rgbd), dataset.GetIntrinsics()};
  auto second_frame =
      FrameType{kSecondFrameIndex, second_observation_data,
                std::move(second_rgbd), dataset.GetIntrinsics()};
  auto tracker = TrackerType{};
  auto tracking_result = tracker.Track(first_frame, second_frame);

  auto ate = ps::metrics::AbsoluteTranslationError(tracking_result.pose,
                                                   gt_relative_pose);

  EXPECT_NEAR(ate, 0.0, kTolerance);
}

TEST(Tracker, TrackMap) {
  constexpr const auto kTolerance = 0.2;
  constexpr const auto kFirstFrameIndex = 0;
  constexpr const auto kSecondFrameIndex = 7;

  auto dataset = CreateTestDataset();
  auto&& gt_poses = dataset.GetGroundTruthPoses();
  auto&& init_pose = gt_poses.front();
  auto observation_creator = ps::CreateDefaultORBObservationCreator();
  auto first_rgbd = dataset[kFirstFrameIndex];
  auto second_rgbd = dataset[kSecondFrameIndex];
  auto first_observation_data =
      observation_creator.Create(dataset[kFirstFrameIndex]);
  auto second_observation_data =
      observation_creator.Create(dataset[kSecondFrameIndex]);
  auto first_frame =
      FrameType{kFirstFrameIndex, first_observation_data, std::move(first_rgbd),
                dataset.GetIntrinsics(), init_pose};
  auto mapper = MapperType{};
  mapper.Initialize(first_frame);
  auto second_frame =
      FrameType{kSecondFrameIndex, second_observation_data,
                std::move(second_rgbd), dataset.GetIntrinsics()};
  auto tracker = TrackerType{};
  auto tracking_result = tracker.TrackMap(second_frame, mapper.GetMap());

  auto ate = ps::metrics::AbsoluteTranslationError(tracking_result.pose,
                                                   gt_poses[kSecondFrameIndex]);

  EXPECT_NEAR(ate, 0.0, kTolerance);
}

}  // namespace prime_slam
