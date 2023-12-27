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

namespace ps = prime_slam;

namespace {

class MockDetector {
 public:
  MockDetector(size_t false_detections_number = 0)
      : false_detections_number_(false_detections_number) {}

  std::vector<ps::observation::Keypoint> Detect(
      const cv::Mat& rgb_image) const {
    cv::Mat locations;  // Will contain the coordinates
    cv::findNonZero(rgb_image, locations);

    std::vector<ps::observation::Keypoint> keypoints;
    keypoints.reserve(locations.total() + false_detections_number_);

    // Output the coordinates
    for (int i = 0; i != locations.total(); ++i) {
      keypoints.emplace_back(Eigen::Vector2d{locations.at<cv::Point>(i).x,
                                             locations.at<cv::Point>(i).y});
    }
    auto height = rgb_image.rows;
    auto width = rgb_image.cols;

    // add points outside the image borders
    for (auto i = 0; i != false_detections_number_; ++i) {
      keypoints.emplace_back(
          Eigen::Vector2d{i % 2 == 0 ? width : -1, i % 2 == 1 ? height : -1});
    }

    return keypoints;
  }

 private:
  size_t false_detections_number_;
};

class MockDescriptor {
 public:
  std::vector<Eigen::VectorXd> Descript(
      const std::vector<ps::observation::Keypoint>& observations,
      const cv::Mat& image) const {
    // The descriptor is simply the coordinate of the point
    std::vector<Eigen::VectorXd> descriptors;
    descriptors.reserve(observations.size());
    std::transform(observations.begin(), observations.end(),
                   std::back_inserter(descriptors), [](auto&& observation) {
                     return observation.GetCoordinates();
                   });
    return descriptors;
  }
};

auto CreateObservationCreator(MockDetector detector) {
  auto fov_filter = prime_slam::observation::PointFOVFilter<
      prime_slam::observation::Keypoint>{};
  auto depth_filter = prime_slam::observation::PointNegativeDepthFilter<
      prime_slam::observation::Keypoint>{};
  auto filter =
      prime_slam::observation::FilterChain<prime_slam::observation::Keypoint>{
          fov_filter, depth_filter};
  auto observation_creator = prime_slam::observation::DefaultObservationCreator<
      MockDetector, MockDescriptor,
      prime_slam::observation::FilterChain<prime_slam::observation::Keypoint>,
      cv::Mat, prime_slam::observation::Keypoint>{detector, MockDescriptor{},
                                                  std::move(filter)};
  return observation_creator;
}

}  // namespace

namespace prime_slam {

TEST(ObservationCreator, CorrectCreation) {
  constexpr const auto kHeight = 16ul;
  constexpr const auto kWidth = 16ul;
  auto points_2d =
      std::vector<Eigen::Vector2d>{{0, 0}, {0, 1}, {15, 3}, {14, 14}};
  auto descriptors = std::vector<Eigen::VectorXd>{};
  descriptors.reserve(points_2d.size());
  std::transform(points_2d.begin(), points_2d.end(),
                 std::back_inserter(descriptors),
                 [](auto&& point_2d) { return point_2d; });
  auto depths = std::vector{0.5, 1.0, 7.0, 13.0};
  auto depth_map = CreateDepthMap(points_2d, depths, kHeight, kWidth);
  cv::Mat image = cv::Mat::zeros(kHeight, kWidth, CV_8U);
  for (auto&& point_2d : points_2d) {
    auto x = static_cast<size_t>(point_2d(0));
    auto y = static_cast<size_t>(point_2d(1));
    image.at<uchar>(y, x) = 1;
  }

  auto rgbd = ps::sensor::RGBDImage{image, depth_map};

  auto observation_creator =
      CreateObservationCreator(MockDetector{/*false_detections_number=*/4});
  auto observations = observation_creator.Create(rgbd);

  EXPECT_EQ(points_2d.size(), observations.Size());
  EXPECT_EQ(points_2d, observations.GetCoordinates());
  EXPECT_EQ(descriptors, observations.GetDescriptors());
}

}  // namespace prime_slam
