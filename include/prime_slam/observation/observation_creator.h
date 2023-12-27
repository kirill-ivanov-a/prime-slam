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
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "prime_slam/observation/description/descriptor_concept.h"
#include "prime_slam/observation/detection/detector_concept.h"
#include "prime_slam/observation/filter/keyobject_filter_concept.h"
#include "prime_slam/observation/observation_batch.h"
#include "prime_slam/sensor/rgbd.h"

namespace prime_slam::observation {

/**
 * @brief Class that creates observations for a given sensor data
 * @tparam DetectorType Type of keyobject detector
 * @tparam DescriptorType Type of keyobject descriptor
 * @tparam KeyobjectFilterType Type of keyobject filter
 * @tparam ImageType Type of sensor data
 * @tparam KeyobjectType Type of keyobject
 */
template <typename DetectorType, typename DescriptorType,
          typename KeyobjectFilterType, typename ImageType,
          typename KeyobjectType>
requires Detector<DetectorType, ImageType, KeyobjectType> &&
    Descriptor<DescriptorType, ImageType, KeyobjectType> &&
    KeyobjectsFilter<KeyobjectFilterType, KeyobjectType>
class DefaultObservationCreator {
 public:
  using ObservationType = typename KeyobjectType::ObservationType;
  using CoordinatesType =
      Eigen::Vector<double,
                    ObservationTraits<ObservationType>::KeyobjectDimension>;

 public:
  /**
   * @brief Creates observation creator
   * @param detector Keyobject detector
   * @param descriptor Keyobject descriptor
   * @param filter Keyobject filter
   */
  DefaultObservationCreator(DetectorType detector, DescriptorType descriptor,
                            std::optional<KeyobjectFilterType> filter)
      : detector_(std::move(detector)),
        descriptor_(std::move(descriptor)),
        filter_(std::move(filter)) {}

  /**
   * @brief Creates batch of observations
   * @param sensor_data Corresponding RGBD image
   * @return observations batch
   */
  ObservationBatch<ObservationType> Create(
      const sensor::RGBDImage& sensor_data) const;

 private:
  DetectorType detector_;
  DescriptorType descriptor_;
  std::optional<KeyobjectFilterType> filter_;
};

// IMPLEMENTATION

template <typename DetectorType, typename DescriptorType,
          typename KeyobjectFilterType, typename ImageType,
          typename KeyobjectType>
requires Detector<DetectorType, ImageType, KeyobjectType> &&
    Descriptor<DescriptorType, ImageType, KeyobjectType> &&
    KeyobjectsFilter<KeyobjectFilterType, KeyobjectType>
inline ObservationBatch<typename KeyobjectType::ObservationType>
DefaultObservationCreator<
    DetectorType, DescriptorType, KeyobjectFilterType, ImageType,
    KeyobjectType>::Create(const sensor::RGBDImage& sensor_data) const {
  auto&& rgb = sensor_data.GetRGB();
  auto keyobjects = detector_.Detect(rgb);
  if (filter_.has_value()) {
    keyobjects = filter_->Apply(std::move(keyobjects), sensor_data);
  }
  auto descriptors = descriptor_.Descript(keyobjects, rgb);

  std::vector<CoordinatesType> coordinates;
  std::vector<double> uncertainties;
  coordinates.reserve(keyobjects.size());
  uncertainties.reserve(keyobjects.size());
  for (auto&& keyobject : keyobjects) {
    coordinates.push_back(keyobject.GetCoordinates());
    uncertainties.push_back(keyobject.GetUncertainty());
  }

  return {std::move(coordinates), std::move(descriptors),
          std::move(uncertainties)};
}

}  // namespace prime_slam::observation
