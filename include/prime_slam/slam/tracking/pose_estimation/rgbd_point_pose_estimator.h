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
#include <random>
#include <utility>
#include <vector>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "prime_slam/projection/point_projector.h"
#include "prime_slam/slam/frame/frame.h"
#include "prime_slam/slam/tracking/pose_estimation/pose_estimator_concept.h"
#include "prime_slam/slam/tracking/pose_estimation/rgbd_pose_estimator.h"

namespace prime_slam::tracking {

using BlockSolverType = g2o::BlockSolver_6_3;
using LinearSolverType =
    g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
using EdgeType = g2o::EdgeSE3ProjectXYZOnlyPose;
using BoolMask = Eigen::Array<bool, Eigen::Dynamic, 1>;
using observation::PointObservationTag;

/**
 * @brief Class for estimating relative and absolute pose using points
 */
template <>
class RGBDPoseEstimator<observation::PointObservationTag> {
 public:
  /**
   * @brief Creates estimator object
   * @param reprojection_threshold Threshold for filtering observations based on
   * reprojection error
   * @param iterations_number Number of iterations of calling the internal g2o
   * optimizer
   * @param optimizer_iterations_number Number of iterations for internal g2o
   * optimizer
   * @param edges_min_number The minimum number of edges in the graph required
   * to continue searching for the optimal pose
   */
  RGBDPoseEstimator(double reprojection_threshold = 20,
                    int iterations_number = 4,
                    int optimizer_iterations_number = 10,
                    int edges_min_number = 20);

  /**
   * @brief Estimates absolute pose of a frame relative to a set of 3D map
   * points
   * @param new_keyframe New selected keyframe
   * @param map_points 3D map points (landmarks)
   * @param matches Matches between new keyframe observations and map landmarks
   * @return Estimated absolute pose
   */
  [[nodiscard]] Eigen::Projective3d EstimatePose(
      const std::vector<Eigen::Vector2d>& observations_2d,
      const Eigen::Projective3d& camera_intrinsics,
      const std::vector<Eigen::Vector3d>& map_points,
      const std::vector<observation::Match>& matches) const;

 private:
  [[nodiscard]] std::unique_ptr<g2o::SparseOptimizer> CreateOptimizer() const;

  [[nodiscard]] EdgeType* CreateEdge(
      Eigen::Vector2d keypoint_coordinates, Eigen::Vector3d map_point,
      const Eigen::Projective3d& intrinsics) const;

 private:
  double reprojection_threshold_;
  int iterations_number_;
  int optimizer_iterations_number_;
  int edges_min_number_;
};

// IMPLEMENTATION

inline RGBDPoseEstimator<observation::PointObservationTag>::RGBDPoseEstimator(
    double reprojection_threshold, int iterations_number,
    int optimizer_iterations_number, int edges_min_number)
    : reprojection_threshold_(reprojection_threshold),
      iterations_number_(iterations_number),
      optimizer_iterations_number_(optimizer_iterations_number),
      edges_min_number_(edges_min_number) {}

inline Eigen::Projective3d
RGBDPoseEstimator<observation::PointObservationTag>::EstimatePose(
    const std::vector<Eigen::Vector2d>& observations_2d,
    const Eigen::Projective3d& camera_intrinsics,
    const std::vector<Eigen::Vector3d>& map_points,
    const std::vector<observation::Match>& matches) const {
  std::vector<size_t> new_keypoints_index;
  std::vector<size_t> map_points_index;
  new_keypoints_index.reserve(matches.size());
  map_points_index.reserve(matches.size());

  for (auto [reference_index, target_index] : matches) {
    new_keypoints_index.push_back(reference_index);
    map_points_index.push_back(target_index);
  }

  std::vector<EdgeType*> edges;
  edges.reserve(matches.size());
  auto optimizer = CreateOptimizer();
  auto vertex = new g2o::VertexSE3Expmap();
  vertex->setId(0);
  vertex->setFixed(false);
  optimizer->addVertex(vertex);

  vertex->setEstimate(
      g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

  for (auto [new_keypoint_index, map_point_index] : matches) {
    auto edge = CreateEdge(observations_2d[new_keypoint_index],
                           map_points[map_point_index], camera_intrinsics);
    edge->setVertex(0, vertex);
    optimizer->addEdge(edge);
    edges.push_back(edge);
  }

  BoolMask inlier_mask = BoolMask::Ones(matches.size());

  for (auto i = 0; i != iterations_number_; ++i) {
    vertex->setEstimate(
        g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));

    optimizer->initializeOptimization();
    optimizer->optimize(optimizer_iterations_number_);

    for (auto j = 0; j != matches.size(); ++j) {
      auto edge = edges[j];
      if (edge->chi2() > reprojection_threshold_ / ((i + 1) * (i + 1))) {
        inlier_mask(j) = false;
        edge->setLevel(1);
      } else {
        inlier_mask(j) = true;
        edge->setLevel(0);
      }
    }
    if (inlier_mask.count() < edges_min_number_) {
      break;
    }
  }
  return Eigen::Projective3d{vertex->estimate().to_homogeneous_matrix()};
}

inline std::unique_ptr<g2o::SparseOptimizer>
RGBDPoseEstimator<observation::PointObservationTag>::CreateOptimizer() const {
  auto linear_solver = std::make_unique<LinearSolverType>();
  auto solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
  auto solver_algorithm =
      new g2o::OptimizationAlgorithmLevenberg(std::move(solver));
  auto optimizer = std::make_unique<g2o::SparseOptimizer>();
  optimizer->setAlgorithm(solver_algorithm);

  return optimizer;
}

inline EdgeType*
RGBDPoseEstimator<observation::PointObservationTag>::CreateEdge(
    Eigen::Vector2d keypoint_coordinates, Eigen::Vector3d map_point,
    const Eigen::Projective3d& intrinsics) const {
  {
    // will be deleted in optimizer
    auto edge = new EdgeType{};
    edge->setMeasurement(std::move(keypoint_coordinates));
    edge->setInformation(Eigen::Matrix2d::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber{});

    edge->fx = intrinsics(0, 0);
    edge->fy = intrinsics(1, 1);
    edge->cx = intrinsics(0, 2);
    edge->cy = intrinsics(1, 2);
    edge->Xw = Eigen::Vector3d{std::move(map_point)};

    return edge;
  }
}

}  // namespace prime_slam::tracking
