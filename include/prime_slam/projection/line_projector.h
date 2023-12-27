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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "prime_slam/projection/point_projector.h"
#include "prime_slam/observation/traits/observation_tag.h"
#include "prime_slam/projection/projector.h"

namespace prime_slam::projection {

template <>
class DefaultProjector<observation::LineObservationTag> {
  // NOT IMPLEMENTED YET
};

}  // namespace prime_slam::projection
