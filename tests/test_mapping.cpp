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

namespace prime_slam {

namespace ps = prime_slam;

using MapType = ps::mapping::DefaultMap<ps::observation::PointObservationTag>;

std::tuple<std::vector<ps::mapping::Landmark>, std::vector<size_t>,
           std::vector<Eigen::Vector3d>, std::vector<Eigen::VectorXd>>
CreateTestLandmarks() {
  auto positions = std::vector<Eigen::Vector3d>{Eigen::Vector3d{0, 0, 0},
                                                Eigen::Vector3d{0, 1, 0},
                                                Eigen::Vector3d{0, 0, 1}};
  auto descriptors = std::vector<Eigen::VectorXd>{Eigen::Vector3d{7, 1, 7},
                                                  Eigen::Vector3d{7, 1, 7},
                                                  Eigen::Vector3d{7, 1, 7}};
  auto ids = std::vector<size_t>{0, 167, 90};
  auto landmarks = std::vector<ps::mapping::Landmark>{};
  for (auto [id, position, descriptor] :
       boost::combine(ids, positions, descriptors)) {
    landmarks.push_back(ps::mapping::Landmark(id, position, descriptor));
  }
  return std::make_tuple(landmarks, ids, positions, descriptors);
}

TEST(Map, InsertLandmark) {
  auto [landmarks, ids, positions, descriptors] = CreateTestLandmarks();
  auto map = MapType{};
  for (auto&& landmark : landmarks) {
    map.InsertLandmark(landmark);
  }

  ASSERT_EQ(map.Size(), landmarks.size());
  ASSERT_EQ(map.GetIDs(), ids);
  ASSERT_EQ(map.GetPositions(), positions);
  ASSERT_EQ(map.GetDescriptors(), descriptors);
}

TEST(Map, At) {
  auto [landmarks, ids, positions, descriptors] = CreateTestLandmarks();
  auto map = MapType{};
  for (auto&& landmark : landmarks) {
    map.InsertLandmark(landmark);
  }

  for (auto&& expected_landmark : landmarks) {
    auto actual_landmark = map.At(expected_landmark.GetID());
    ASSERT_EQ(actual_landmark.GetPosition(), expected_landmark.GetPosition());
    ASSERT_EQ(actual_landmark.GetDescriptor(),
              expected_landmark.GetDescriptor());
    ASSERT_EQ(actual_landmark.GetID(), expected_landmark.GetID());
  }
}

TEST(Map, RemoveLandmarks) {
  auto [landmarks, ids, positions, descriptors] = CreateTestLandmarks();
  auto map = MapType{};
  for (auto&& landmark : landmarks) {
    map.InsertLandmark(landmark);
  }
  auto first_id = landmarks.front().GetID();
  auto last_id = landmarks.back().GetID();
  map.RemoveLandmarks({first_id, last_id});

  for (auto&& expected_landmark : landmarks) {
    auto id = expected_landmark.GetID();
    if (id == first_id || id == last_id) {
      continue;
    }
    auto actual_landmark = map.At(id);
    ASSERT_EQ(actual_landmark.GetPosition(), expected_landmark.GetPosition());
    ASSERT_EQ(actual_landmark.GetDescriptor(),
              expected_landmark.GetDescriptor());
    ASSERT_EQ(actual_landmark.GetID(), expected_landmark.GetID());
  }
  ASSERT_EQ(map.Size(), landmarks.size() - 2);
}

}  // namespace prime_slam
