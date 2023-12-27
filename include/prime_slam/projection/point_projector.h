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

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "prime_slam/observation/traits/observation_tag.h"
#include "prime_slam/projection/projector.h"

namespace prime_slam::projection {

/**
 * @brief Class using for points projection operations
 */
template <>
class DefaultProjector<observation::PointObservationTag> {
 public:
  /**
   * @brief Transforms 3D points using given transformation
   * @param points_3d 3D points to transform
   * @param transformation Transformation object
   * @return Transformed 3D points
   */
  [[nodiscard]] std::vector<Eigen::Vector3d> Transform(
      const std::vector<Eigen::Vector3d>& points_3d,
      const Eigen::Projective3d& transformation) const;

  /**
   * @brief Transforms 3D point using given transformation
   * @param point_3d 3D point to transform
   * @param transformation Transformation object
   * @return Transformed 3D point
   */
  [[nodiscard]] Eigen::Vector3d Transform(
      const Eigen::Vector3d& point_3d,
      const Eigen::Projective3d& transformation_matrix) const;

  /**
   * @brief Projects given 3D points into image frame system
   * @param points_3d Points 3D in world coordinate system to project
   * @param intrinsics Camera intrinsics
   * @param extrinsics Camera extrinsics
   * @return Projected 2D points
   */
  [[nodiscard]] std::vector<Eigen::Vector2d> Project(
      const std::vector<Eigen::Vector3d>& points_3d,
      const Eigen::Projective3d& intrinsics,
      const Eigen::Projective3d& extrinsics) const;

  /**
   * @brief Projects given 3D point into image frame system
   * @param points_3d Point 3D in world coordinate system to project
   * @param intrinsics Camera intrinsics
   * @param extrinsics Camera extrinsics
   * @return Projected 2D point
   */
  [[nodiscard]] Eigen::Vector2d Project(
      const Eigen::Vector3d& point_3d, const Eigen::Projective3d& intrinsics,
      const Eigen::Projective3d& extrinsics) const;

  /**
   * @brief Back projects 2D points from camera to world
   * @param points_2d Points 2D in camera coordinate system
   * @param intrinsics Camera intrinsics
   * @param depth_map Corresponding depth map
   * @param extrinsics Camera extrinsics
   * @return Back projected 3D points
   */
  [[nodiscard]] std::vector<Eigen::Vector3d> BackProject(
      const std::vector<Eigen::Vector2d>& points_2d,
      const Eigen::Projective3d& intrinsics, const Eigen::MatrixXd& depth_map,
      const Eigen::Projective3d& extrinsics) const;

  /**
   * @brief Back projects 2D point from camera to world
   * @param points_2d Point 2D in camera coordinate system
   * @param intrinsics Camera intrinsics
   * @param depth_map Corresponding depth map
   * @param extrinsics Camera extrinsics
   * @return Back projected 3D point
   */
  [[nodiscard]] Eigen::Vector3d BackProject(
      const Eigen::Vector2d& point_2d, const Eigen::Projective3d& intrinsics,
      const Eigen::MatrixXd& depth_map,
      const Eigen::Projective3d& extrinsics) const;

 private:
  [[nodiscard]] Eigen::Vector2d ProjectImpl(
      const Eigen::Vector3d& point_3d,
      const Eigen::Projective3d& projection_matrix) const;

  [[nodiscard]] Eigen::Vector3d BackProjectImpl(
      const Eigen::Vector2d& point_2d, const Eigen::Projective3d& intrinsics,
      const Eigen::MatrixXd& depth_map,
      const Eigen::Projective3d& extrinsics_inv) const;
};

// IMPLEMENTATION

inline std::vector<Eigen::Vector3d>
DefaultProjector<observation::PointObservationTag>::Transform(
    const std::vector<Eigen::Vector3d>& points_3d,
    const Eigen::Projective3d& transformation) const {
  std::vector<Eigen::Vector3d> transformed_points;
  transformed_points.reserve(points_3d.size());
  std::transform(points_3d.begin(), points_3d.end(),
                 std::back_inserter(transformed_points),
                 [this, &transformation](auto&& point) {
                   return Transform(point, transformation);
                 });
  return transformed_points;
}

inline Eigen::Vector3d
DefaultProjector<observation::PointObservationTag>::Transform(
    const Eigen::Vector3d& point_3d,
    const Eigen::Projective3d& transformation) const {
  auto transformed_point =
      (transformation * point_3d.homogeneous()).hnormalized();
  return transformed_point;
}

inline std::vector<Eigen::Vector2d>
DefaultProjector<observation::PointObservationTag>::Project(
    const std::vector<Eigen::Vector3d>& points_3d,
    const Eigen::Projective3d& intrinsics,
    const Eigen::Projective3d& extrinsics) const {
  auto projection_matrix = intrinsics * extrinsics;
  std::vector<Eigen::Vector2d> projected_points;
  projected_points.reserve(points_3d.size());
  std::transform(points_3d.begin(), points_3d.end(),
                 std::back_inserter(projected_points),
                 [this, &projection_matrix](auto&& point) {
                   return ProjectImpl(point, projection_matrix);
                 });
  return projected_points;
}

inline Eigen::Vector2d
DefaultProjector<observation::PointObservationTag>::Project(
    const Eigen::Vector3d& point_3d, const Eigen::Projective3d& intrinsics,
    const Eigen::Projective3d& extrinsics) const {
  auto projection_matrix = intrinsics * extrinsics;
  return ProjectImpl(point_3d, projection_matrix);
}

inline std::vector<Eigen::Vector3d>
DefaultProjector<observation::PointObservationTag>::BackProject(
    const std::vector<Eigen::Vector2d>& points_2d,
    const Eigen::Projective3d& intrinsics, const Eigen::MatrixXd& depth_map,
    const Eigen::Projective3d& extrinsics) const {
  auto extrinsics_inv = extrinsics.inverse();
  std::vector<Eigen::Vector3d> back_projected_points;
  back_projected_points.reserve(points_2d.size());
  std::transform(
      points_2d.begin(), points_2d.end(),
      std::back_inserter(back_projected_points),
      [this, &intrinsics, &depth_map, &extrinsics_inv](auto&& point) {
        return BackProjectImpl(point, intrinsics, depth_map, extrinsics_inv);
      });
  return back_projected_points;
}

inline Eigen::Vector3d
DefaultProjector<observation::PointObservationTag>::BackProject(
    const Eigen::Vector2d& point_2d, const Eigen::Projective3d& intrinsics,
    const Eigen::MatrixXd& depth_map,
    const Eigen::Projective3d& extrinsics) const {
  return BackProjectImpl(point_2d, intrinsics, depth_map, extrinsics.inverse());
}

inline Eigen::Vector2d
DefaultProjector<observation::PointObservationTag>::ProjectImpl(
    const Eigen::Vector3d& point_3d,
    const Eigen::Projective3d& projection_matrix) const {
  auto projected_point = (projection_matrix * point_3d.homogeneous());
  auto normalized_projected_point = (projected_point / projected_point(2));
  return normalized_projected_point(Eigen::seqN(0, 2));
}

inline Eigen::Vector3d
DefaultProjector<observation::PointObservationTag>::BackProjectImpl(
    const Eigen::Vector2d& point_2d, const Eigen::Projective3d& intrinsics,
    const Eigen::MatrixXd& depth_map,
    const Eigen::Projective3d& extrinsics_inv) const {
  auto fx = intrinsics(0, 0);
  auto fy = intrinsics(1, 1);
  auto cx = intrinsics(0, 2);
  auto cy = intrinsics(1, 2);

  auto x = point_2d(0);
  auto y = point_2d(1);
  auto uf = (x - cx) / fx;
  auto vf = (y - cy) / fy;
  auto depth = depth_map(static_cast<int>(y), static_cast<int>(x));

  auto x_3d = uf * depth;
  auto y_3d = vf * depth;
  auto z_3d = depth;
  Eigen::Vector3d point3_d{x_3d, y_3d, z_3d};

  return Transform(point3_d, extrinsics_inv);
}

}  // namespace prime_slam::projection
