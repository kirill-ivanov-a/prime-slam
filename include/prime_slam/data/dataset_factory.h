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

#include <filesystem>
#include <memory>
#include <unordered_map>

#include "prime_slam/data/datasets/data_format.h"
#include "prime_slam/data/datasets/icl_nuim_dataset.h"
#include "prime_slam/data/datasets/icl_nuim_tum_format_dataset.h"
#include "prime_slam/data/datasets/rgbd_dataset_interface.h"
#include "prime_slam/data/datasets/tum_rgbd_dataset.h"
#include "prime_slam/data/rgbd_dataset.h"

namespace prime_slam {

namespace fs = std::filesystem;

/**
 * @brief Factory for creation dataset of given format
 */
class DatasetFactory {
 public:
  /**
   * @brief Creates dataset of given format
   * @param format Desired dataset format
   * @param data_path Path to the folder with data in corresponding format
   * @return dataset object
   */
  static RGBDDataset Create(DataFormat format, const fs::path& data_path);

 private:
  static std::unordered_map<
      DataFormat,
      std::function<std::unique_ptr<IRGBDDataset>(const fs::path& data_path)>>
      constructors;
};

}  // namespace prime_slam
