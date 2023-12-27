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

#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "prime_slam/slam/mapping/map.h"

namespace prime_slam::visualization {

/**
 * @brief Visualizes landmarks of map
 * @tparam MapImpl Concrete map type
 * @param map Map object
 */
template <mapping::Map MapImpl>
void VisualizeMap(const MapImpl& map);

// IMPLEMENTATION

template <std::derived_from<observation::ObservationTag> ObservationType>
class GeometryCreator;

template <>
class GeometryCreator<observation::PointObservationTag> {
 public:
  using LandmarkCoordinatesType =
      Eigen::Vector<double,
                    observation::ObservationTraits<
                        observation::PointObservationTag>::LandmarkDimension>;

  static std::shared_ptr<const open3d::geometry::Geometry> Create(
      const std::vector<LandmarkCoordinatesType>& points) {
    open3d::geometry::PointCloud point_cloud(points);
    return std::make_shared<const open3d::geometry::PointCloud>(point_cloud);
  }
};

template <mapping::Map MapImpl>
inline void VisualizeMap(const MapImpl& map) {
  auto&& landmark_positions = map.GetPositions();
  std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
  geometries.push_back(
      GeometryCreator<typename MapImpl::ObservationType>::Create(
          landmark_positions));
  open3d::visualization::DrawGeometries(geometries);
}

}  // namespace prime_slam::visualization
