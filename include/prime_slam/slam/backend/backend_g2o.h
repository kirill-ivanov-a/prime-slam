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

#include <cstdint>
#include <memory>
#include <unordered_map>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <Eigen/Geometry>

#include "prime_slam/slam/backend/backend_concept.h"
#include "prime_slam/slam/backend/backend_result.h"
#include "prime_slam/slam/graph/factor_graph.h"

namespace prime_slam::backend {

/**
 * @brief Backend using g2o library
 */
class G2OPointBackend final {
 public:
  /**
   * @brief Constructs a backend object
   * @param intrinsics intrinsics of camera
   * @param optimizer_iterations_number Number of iterations for internal g2o
   * optimizer
   * @param bf To simulate stereo viewing, the size of the baseline multiplied
   * by the focal length for x
   */
  G2OPointBackend(Eigen::Projective3d intrinsics,
                  uint64_t optimizer_iterations_number = 25, double bf = 400)
      : fx_(intrinsics(0, 0)),
        fy_(intrinsics(1, 1)),
        cx_(intrinsics(0, 2)),
        cy_(intrinsics(1, 2)),
        bf_(bf),
        optimizer_iterations_number_(optimizer_iterations_number) {}

  /**
   * @brief Optimizes the passed factor graph
   * @param graph Factor graph to optimize
   * @param verbose Display convergence information
   * @return Optimized poses and landmarks
   */
  [[nodiscard]] BackendResult Optimize(const factor_graph::FactorGraph& graph,
                                       bool verbose = false) const;

 private:
  [[nodiscard]] g2o::VertexSE3Expmap* CreatePoseVertex(
      const factor_graph::PoseNode& pose_node) const;

  [[nodiscard]] g2o::VertexPointXYZ* CreateLandmarkVertex(
      const factor_graph::LandmarkNode& landmark_node) const;

  g2o::EdgeStereoSE3ProjectXYZ* CreateEdge(
      g2o::OptimizableGraph::Vertex* landmark_vertex,
      g2o::OptimizableGraph::Vertex* pose_vertex,
      Eigen::Vector3d observation) const;

  [[nodiscard]] std::unique_ptr<g2o::SparseOptimizer> CreateOptimizer() const;

 private:
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double bf_;
  uint64_t optimizer_iterations_number_;
};

}  // namespace prime_slam::backend
