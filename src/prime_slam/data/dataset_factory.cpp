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

#include "prime_slam/data/dataset_factory.h"

namespace prime_slam {

RGBDDataset DatasetFactory::Create(DataFormat format,
                                   const fs::path& data_path) {
  return RGBDDataset{constructors[format](data_path)};
}

std::unordered_map<DataFormat, std::function<std::unique_ptr<IRGBDDataset>(
                                   const fs::path& data_path)>>
    DatasetFactory::constructors = {
        {DataFormat::ICL,
         [](const fs::path& data_path) {
           return std::make_unique<ICLNUIMDataset>(data_path);
         }},
        {DataFormat::ICL_TUM,
         [](const fs::path& data_path) {
           return std::make_unique<ICLNUIMTUMFormatDataset>(data_path);
         }},
        {DataFormat::TUM, [](const fs::path& data_path) {
           return std::make_unique<TUMRGBDDataset>(data_path);
         }}};

}  // namespace prime_slam
