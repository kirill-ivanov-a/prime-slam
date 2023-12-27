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
using ps::metrics::AbsoluteTranslationError;
using ps::observation::PointObservationTag;
using ps::projection::DefaultProjector;
using Projector = ps::projection::DefaultProjector<PointObservationTag>;

factor_graph::FactorGraph GenerateSyntheticGraph(const RGBDDataset& dataset,
                                                 size_t first_frame_index,
                                                 size_t second_frame_index) {
  constexpr const auto kPointsNumber = 100;
  auto first_rgbd = dataset[first_frame_index];
  auto second_rgbd = dataset[second_frame_index];
  auto image_height = first_rgbd.GetHeight();
  auto image_width = first_rgbd.GetWidth();
  auto&& gt_poses = dataset.GetGroundTruthPoses();
  Projector projector;

  auto first_frame_points_2d = GenerateRandom2DPoints(
      kPointsNumber, static_cast<double>(image_width) - 1,
      static_cast<double>(image_height) - 1);  // each element: from -1 to 1
  auto&& first_frame_absolute_pose = gt_poses[0];
  auto&& second_frame_absolute_pose = gt_poses[1];
  auto&& intrinsics = dataset.GetIntrinsics();
  auto world_points =
      projector.BackProject(first_frame_points_2d, intrinsics,
                            first_rgbd.GetDepth(), first_frame_absolute_pose);
  auto second_frame_points_2d =
      projector.Project(world_points, intrinsics, second_frame_absolute_pose);

  factor_graph::FactorGraph graph;
  graph.AddPoseNode(0, first_frame_absolute_pose);
  graph.AddPoseNode(1, Eigen::Projective3d::Identity());

  for (auto landmark_id = 0; landmark_id != kPointsNumber; ++landmark_id) {
    graph.AddLandmarkNode(landmark_id, world_points[landmark_id]);
    graph.AddObservationFactor(/*pose_id=*/0, landmark_id,
                               first_frame_points_2d[landmark_id], first_rgbd);
    auto second_point_2d = second_frame_points_2d[landmark_id];
    if (IsInImage(second_point_2d, image_width, image_height)) {
      graph.AddObservationFactor(/*pose_id=*/1, landmark_id, second_point_2d,
                                 second_rgbd);
    }
  }

  return graph;
}

TEST(Backend, GraphOptimized) {
  constexpr const auto kTolerance = 0.3;
  auto dataset = CreateTestDataset();
  auto&& gt_poses = dataset.GetGroundTruthPoses();
  auto&& intrinsics = dataset.GetIntrinsics();
  auto first_frame_index = 0ul;
  auto second_frame_index = 1ul;
  auto graph =
      GenerateSyntheticGraph(dataset, first_frame_index, second_frame_index);

  auto backend = backend::G2OPointBackend{intrinsics};
  auto optimized_poses = backend.Optimize(graph).optimized_poses;
  auto first_frame_ate = AbsoluteTranslationError(
      optimized_poses[first_frame_index].pose, gt_poses[first_frame_index]);
  auto second_frame_ate = AbsoluteTranslationError(
      optimized_poses[second_frame_index].pose, gt_poses[second_frame_index]);

  EXPECT_NEAR(first_frame_ate, 0.0, kTolerance);
  EXPECT_NEAR(second_frame_ate, 0.0, kTolerance);
}

TEST(Backend, BetterResult) {
  auto dataset = CreateTestDataset();
  auto&& gt_poses = dataset.GetGroundTruthPoses();
  auto slam =
      ps::CreateDefaultORBPipeline(dataset.GetIntrinsics(), gt_poses.front());
  auto frontend =
      ps::CreateDefaultORBFrontend(dataset.GetIntrinsics(), gt_poses.front());
  for (auto&& data : dataset) {
    slam.ProcessSensorData(data);
    frontend.TryInsertSensorData(data);
  }
  auto est_poses = frontend.GetTrajectory();
  auto optimized_est_poses = slam.GetTrajectory();

  auto max_ate = 0.0;
  auto max_optimized_ate = 0.0;
  for (auto&& [gt_pose, est_pose, optimized_est_pose] :
       boost::combine(gt_poses, est_poses, optimized_est_poses)) {
    auto ate = AbsoluteTranslationError(est_pose, gt_pose);
    auto optimized_ate = AbsoluteTranslationError(est_pose, optimized_est_pose);
    max_ate = std::max(max_ate, ate);
    max_optimized_ate = std::max(max_optimized_ate, optimized_ate);
  }

  EXPECT_LE(max_optimized_ate, max_ate);
}

}  // namespace prime_slam
