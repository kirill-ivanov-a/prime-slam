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

#include <filesystem>

#include <gtest/gtest.h>
#include <prime_slam/prime_slam.h>

#include "data_paths.h"

namespace prime_slam {

namespace fs = std::filesystem;

TEST(InvalidICLNUIMTUM, NoPosesFile) {
  auto base_path = fs::path(kInvalidICLTUMPath);
  EXPECT_THROW({ prime_slam::ICLNUIMTUMFormatDataset dataset{base_path}; },
               fs::filesystem_error);
}

TEST(InvalidICLNUIMTUM, WrongPath) {
  auto base_path = fs::path("wrong");
  EXPECT_THROW({ prime_slam::ICLNUIMTUMFormatDataset dataset{base_path}; },
               std::invalid_argument);
}

TEST(InvalidICLNUIM, WrongPosesFile) {
  auto base_path = fs::path(kInvalidICLPath);
  EXPECT_THROW({ prime_slam::ICLNUIMDataset dataset{base_path}; },
               std::invalid_argument);
}

TEST(InvalidICLNUIM, WrongPath) {
  auto base_path = fs::path("wrong");
  EXPECT_THROW({ prime_slam::ICLNUIMDataset dataset{base_path}; },
               fs::filesystem_error);
}

TEST(InvalidTUM, InvalidFileNames) {
  auto base_path = fs::path(kInvalidTUMPath);
  EXPECT_THROW({ prime_slam::TUMRGBDDataset dataset{base_path}; },
               std::invalid_argument);
}

TEST(InvalidTUM, WrongPath) {
  auto base_path = fs::path("wrong");
  EXPECT_THROW({ prime_slam::TUMRGBDDataset dataset{base_path}; },
               std::invalid_argument);
}

}  // namespace prime_slam
