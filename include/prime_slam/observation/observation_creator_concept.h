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

#include <concepts>
#include <memory>

#include "prime_slam/observation/description/descriptor_concept.h"
#include "prime_slam/observation/detection/detector_concept.h"
#include "prime_slam/observation/filter/keyobject_filter_concept.h"
#include "prime_slam/observation/observation_batch.h"
#include "prime_slam/observation/traits/observation_tag.h"
#include "prime_slam/sensor/rgbd.h"

namespace prime_slam::observation {

/**
 * @brief Concept of observation creator
 * @tparam ObservationCreatorImpl
 */
template <typename ObservationCreatorImpl>
concept ObservationCreator =
    std::derived_from<typename ObservationCreatorImpl::ObservationType,
                      prime_slam::observation::ObservationTag> &&
    std::movable<ObservationCreatorImpl> &&
    requires(ObservationCreatorImpl creator) {
  typename ObservationCreatorImpl::ObservationType;
  // Creates batch of observations
  {
    creator.Create(std::declval<sensor::RGBDImage>())
    } -> std::convertible_to<
        ObservationBatch<typename ObservationCreatorImpl::ObservationType>>;
};

}  // namespace prime_slam::observation
