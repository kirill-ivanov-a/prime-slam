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

#ifndef TEST_DATA_DIR
#define TEST_DATA_DIR "."
#endif

namespace prime_slam {

const auto kBaseDataPath = fs::path{TEST_DATA_DIR};

const auto kTestDatasetPath = kBaseDataPath / "lr_kt2_reduced";

const auto kInvalidDatasetPath = kBaseDataPath / "invalid";

const auto kInvalidICLPath = kInvalidDatasetPath / "icl";

const auto kInvalidICLTUMPath = kInvalidDatasetPath / "icl_tum";

const auto kInvalidTUMPath = kInvalidDatasetPath / "tum";

}  // namespace prime_slam
