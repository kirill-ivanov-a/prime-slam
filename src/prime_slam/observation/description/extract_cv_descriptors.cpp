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

#include "extract_cv_descriptors.h"

namespace prime_slam {

std::vector<Eigen::VectorXd> ExtractCVDescriptors(
    const cv::Mat& descriptors_mat) {
  std::vector<Eigen::VectorXd> descriptors;

  for (auto i = 0; i != descriptors_mat.rows; ++i) {
    auto row = descriptors_mat.row(i);
    Eigen::VectorXd descriptor(row.cols);
    std::copy(row.begin<uint8_t>(), row.end<uint8_t>(), descriptor.begin());
    descriptors.push_back(std::move(descriptor));
  }

  return descriptors;
}

}  // namespace prime_slam
