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

#include <Eigen/Core>

#include "prime_slam/observation/keyobject_concept.h"
#include "prime_slam/observation/observation.h"
#include "prime_slam/observation/traits/observation_traits.h"

namespace prime_slam::observation {

/**
 * @brief Class representing batch of observation
 * @tparam ObservationType Type of observations
 */
template <std::derived_from<ObservationTag> ObservationType>
class ObservationBatch final {
 public:
  using CoordinatesType =
      Eigen::Vector<double,
                    ObservationTraits<ObservationType>::KeyobjectDimension>;

  /**
   * @brief Creates default batch
   */
  ObservationBatch() = default;

  /**
   * @brief Creates batch from coordinates, descriptors, and uncertainties
   * @param coordinates Coordinates of observations
   * @param descriptors Descriptors of observations
   * @param uncertainties Uncertainties of observations
   */
  ObservationBatch(std::vector<CoordinatesType> coordinates,
                   std::vector<Eigen::VectorXd> descriptors,
                   std::vector<double> uncertainties)
      : coordinates_(std::move(coordinates)),
        descriptors_(std::move(descriptors)),
        uncertainties_(std::move(uncertainties)),
        size_(coordinates_.size()) {}

  /**
   * @brief Returns number of observations
   * @return size of a batch
   */
  [[nodiscard]] size_t Size() const noexcept { return size_; }

  /**
   * @brief Returns observation by its index
   * @param index Index of desired observation
   * @return Observation at given index
   */
  Observation<ObservationType> operator[](size_t index) const {
    return {coordinates_[index], descriptors_[index], uncertainties_[index]};
  }

  /**
   * @brief Returns descriptors of observations
   * @return Descriptors of observations
   */
  [[nodiscard]] const std::vector<Eigen::VectorXd>& GetDescriptors()
      const& noexcept {
    return descriptors_;
  }

  /**
   * @brief Returns coordinates of observations
   * @return Coordinates of observations
   */
  [[nodiscard]] const std::vector<CoordinatesType>& GetCoordinates()
      const& noexcept {
    return coordinates_;
  }

  /**
   * @brief Returns uncertainties of observations
   * @return Coordinates of observations
   */
  [[nodiscard]] const std::vector<double>& GetUncertainties() const& noexcept {
    return uncertainties_;
  }

 private:
  const std::vector<Eigen::VectorXd> descriptors_;
  const std::vector<CoordinatesType> coordinates_;
  const std::vector<double> uncertainties_;
  const size_t size_{0};
};

}  // namespace prime_slam::observation
