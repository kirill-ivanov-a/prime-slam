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

#include <memory>
#include <utility>
#include <vector>

#include "prime_slam/data/dataset_factory.h"
#include "prime_slam/data/dataset_iterator.h"
#include "prime_slam/data/datasets/rgbd_dataset_interface.h"

namespace prime_slam {

/**
 * @brief Non-virtual dataset interface
 */
class RGBDDataset final {
 public:
  /**
   * @brief DatasetFactory calls private constructor
   */
  friend class DatasetFactory;

  /**
   * @brief Cannot be created by default
   */
  RGBDDataset() = delete;

  /**
   * @brief Returns the RGBD Image at the corresponding index
   * @param index Index of dataset RGBD Image
   * @return RGBD Image
   */
  sensor::RGBDImage operator[](size_t index) const {
    return impl_->operator[](index);
  }

  /**
   * @brief Returns size of the dataset
   * @return Dataset size
   */
  [[nodiscard]] size_t Size() const noexcept { return impl_->Size(); }

  /**
   * @brief Returns groundtruth poses for each frame (RGBD Image)
   * @return Groundtruth poses
   */
  [[nodiscard]] const std::vector<Eigen::Projective3d>& GetGroundTruthPoses()
      const& noexcept {
    return impl_->GetGroundTruthPoses();
  }

  /**
   * @brief Returns corresponding camera intrinsics
   * @return Camera intrinsics
   */
  [[nodiscard]] const Eigen::Projective3d& GetIntrinsics() const& noexcept {
    return impl_->GetIntrinsics();
  }

  /**
   * @brief Returns begin dataset iterator
   * @return Begin dataset iterator
   */
  DatasetIterator begin() const& noexcept {  // NOLINT
    return DatasetIterator{*impl_, 0};
  }

  /**
   * @brief Returns end dataset iterator
   * @return End dataset iterator
   */
  DatasetIterator end() const& noexcept {  // NOLINT
    return DatasetIterator{*impl_, Size()};
  }

 private:
  /**
   * @brief Private constructor for DatasetFactory
   * @param impl Pointer to concrete dataset implementation
   */
  explicit RGBDDataset(std::unique_ptr<IRGBDDataset> impl)
      : impl_(std::move(impl)) {}

 private:
  std::unique_ptr<IRGBDDataset> impl_;
};

}  // namespace prime_slam
