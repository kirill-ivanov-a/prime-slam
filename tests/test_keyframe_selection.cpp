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

using FrameType = ps::Frame<ps::observation::PointObservationTag>;

TEST(EveryNthKeyframeSelector, CorrectSelection) {
  constexpr const auto kStep = 2;
  constexpr const auto kFramesNumber = 20;
  auto selector =
      ps::EveryNthKeyframeSelector<ps::observation::PointObservationTag>{kStep};
  std::vector<FrameType> frames;
  frames.reserve(kFramesNumber);
  std::vector<bool> expected_selected;
  expected_selected.reserve(kFramesNumber);
  for (auto i = 0ul; i != kFramesNumber; ++i) {
    frames.push_back(FrameType{i});
    expected_selected.push_back(i % kStep == 1);
  }
  std::vector<bool> actual_selected;
  actual_selected.reserve(frames.size());
  std::transform(
      frames.begin(), frames.end(), std::back_inserter(actual_selected),
      [&selector](auto&& frame) { return selector.IsSelected(frame); });
  EXPECT_EQ(expected_selected, actual_selected);
}

}  // namespace prime_slam
