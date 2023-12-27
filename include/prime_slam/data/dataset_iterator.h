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

#include <iterator>
#include <utility>

#include "prime_slam/data/datasets/rgbd_dataset_interface.h"
#include "prime_slam/sensor/rgbd.h"

namespace prime_slam {

/**
 * @brief Iterator for dataset
 */
class DatasetIterator {
 public:
  using iterator_category = std::input_iterator_tag;  // NOLINT
  using value_type = sensor::RGBDImage;               // NOLINT
  using difference_type = size_t;                     // NOLINT
  using reference = value_type&;                      // NOLINT
  /**
   * No arrow operator
   */
  using pointer = void;  // NOLINT

 public:
  /**
   * @brief Creates iterator from data ref and starting index
   * @param data Reference to data
   * @param start_index Starting index
   */

  explicit DatasetIterator(const IRGBDDataset& data, size_t start_index = 0)
      : data_(data), current_index_(start_index) {}

  /**
   * @brief Equality comparison operator
   * @param rhs Other dataset iterator
   * @return True if data references and current indices are equal
   */
  bool operator==(DatasetIterator rhs) const noexcept {
    return (&data_ == &rhs.data_) && (current_index_ == rhs.current_index_);
  }

  /**
   * @brief Inequality comparison operator
   * @param rhs Other dataset iterator
   * @return Negation of equality comparison operator result
   */
  bool operator!=(DatasetIterator rhs) const noexcept {
    return !(*this == rhs);
  }

  /**
   * @brief Prefix increment
   *
   * Increments current index
   * @return Reference to the same object
   */
  DatasetIterator& operator++() {
    ++current_index_;
    return *this;
  }

  /**
   * @brief Postfix increment operator
   *
   * Increments current index
   * @return New iterator with incremented current index
   */
  DatasetIterator operator++(int) {
    auto tmp{*this};
    ++current_index_;
    return tmp;
  }

  /**
   * @brief Dereference operator
   * @return RGBD Image object
   */
  value_type operator*() const { return data_[current_index_]; }

 private:
  size_t current_index_;
  const IRGBDDataset& data_;
};

}  // namespace prime_slam
