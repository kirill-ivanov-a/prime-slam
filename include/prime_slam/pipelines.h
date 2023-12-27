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

#include <utility>

#include "prime_slam/prime_slam.h"
#include "prime_slam/slam/backend/backend_g2o.h"
#include "prime_slam/slam/frame/keyframe_selection/every_nth_keyframe_selector.h"
#include "prime_slam/slam/frontend/tracking_frontend.h"
#include "prime_slam/slam/slam.h"

namespace prime_slam {

inline auto CreateDefaultORBObservationCreator() {
  auto detector = prime_slam::observation::ORB{};
  auto descriptor = prime_slam::observation::ORBDescriptor{};
  auto fov_filter = prime_slam::observation::PointFOVFilter<
      prime_slam::observation::OpenCVKeypoint>{};
  auto depth_filter = prime_slam::observation::PointNegativeDepthFilter<
      prime_slam::observation::OpenCVKeypoint>{};
  auto filter = prime_slam::observation::FilterChain<
      prime_slam::observation::OpenCVKeypoint>{fov_filter, depth_filter};
  auto observation_creator = prime_slam::observation::DefaultObservationCreator<
      prime_slam::observation::ORB, prime_slam::observation::ORBDescriptor,
      prime_slam::observation::FilterChain<
          prime_slam::observation::OpenCVKeypoint>,
      cv::Mat, prime_slam::observation::OpenCVKeypoint>{
      std::move(detector), std::move(descriptor), std::move(filter)};
  return observation_creator;
}

inline auto CreateDefaultORBFrontend(
    Eigen::Projective3d intrinsics,
    Eigen::Projective3d initial_pose = Eigen::Projective3d::Identity(),
    size_t frame_step = 1) {
  auto keyframe_selector = prime_slam::EveryNthKeyframeSelector<
      prime_slam::observation::ORB::ObservationType>(frame_step);
  auto observation_creator = CreateDefaultORBObservationCreator();

  auto frontend = prime_slam::frontend::TrackingFrontend{
      std::move(initial_pose), std::move(intrinsics),
      std::move(observation_creator), keyframe_selector};

  return frontend;
}

inline auto CreateDefaultORBPipeline(
    Eigen::Projective3d intrinsics,
    Eigen::Projective3d initial_pose = Eigen::Projective3d::Identity(),
    size_t frame_step = 1) {
  auto frontend =
      CreateDefaultORBFrontend(intrinsics, initial_pose, frame_step);
  auto backend = prime_slam::backend::G2OPointBackend{intrinsics};

  return prime_slam::SLAM(std::move(frontend), std::move(backend));
}

}  // namespace prime_slam
