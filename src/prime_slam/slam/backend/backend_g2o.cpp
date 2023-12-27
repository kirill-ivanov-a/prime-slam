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

#include "prime_slam/slam/backend/backend_g2o.h"

#include <unordered_map>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <Eigen/Geometry>
#include <boost/range/adaptor/reversed.hpp>

namespace prime_slam::backend {

using BlockSolverType = g2o::BlockSolver_6_3;
using LinearSolverType =
    g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

BackendResult G2OPointBackend::Optimize(const factor_graph::FactorGraph& graph,
                                        bool verbose) const {
  auto optimizer = CreateOptimizer();
  auto&& pose_nodes = graph.GetPoseNodes();
  auto&& landmark_nodes = graph.GetLandmarkNodes();
  auto&& observation_factors = graph.GetObservationFactors();
  size_t vertex_counter = 0;

  std::unordered_map<size_t, g2o::VertexSE3Expmap*> pose_id_to_vertex;
  pose_id_to_vertex.reserve(pose_nodes.size());
  std::unordered_map<size_t, g2o::VertexPointXYZ*> landmark_id_to_vertex;
  landmark_id_to_vertex.reserve(landmark_nodes.size());

  for (auto&& pose_node : pose_nodes) {
    auto vertex = CreatePoseVertex(pose_node);
    vertex->setId(vertex_counter++);
    optimizer->addVertex(vertex);
    pose_id_to_vertex[pose_node.GetID()] = vertex;
  }

  for (auto&& landmark_node : landmark_nodes) {
    auto vertex = CreateLandmarkVertex(landmark_node);
    vertex->setId(vertex_counter++);
    optimizer->addVertex(vertex);
    landmark_id_to_vertex[landmark_node.GetID()] = vertex;
  }

  for (auto&& observation_factor : observation_factors) {
    auto landmark_id = observation_factor.GetLandmarkNodeID();
    auto pose_id = observation_factor.GetPoseNodeID();
    auto landmark_vertex = landmark_id_to_vertex[landmark_id];
    auto pose_vertex = pose_id_to_vertex[pose_id];
    auto observation = observation_factor.GetObservation();
    auto edge = CreateEdge(landmark_vertex, pose_vertex, observation);
    optimizer->addEdge(edge);
  }

  optimizer->setVerbose(verbose);
  optimizer->initializeOptimization();
  optimizer->optimize(optimizer_iterations_number_);

  BackendResult result;
  result.optimized_poses.reserve(pose_nodes.size());
  result.optimized_positions.reserve(landmark_nodes.size());

  std::transform(pose_id_to_vertex.begin(), pose_id_to_vertex.end(),
                 std::back_inserter(result.optimized_poses), [](auto&& item) {
                   return OptimizedPose{
                       item.first,
                       Eigen::Projective3d{
                           item.second->estimate().to_homogeneous_matrix()}};
                 });

  std::transform(
      landmark_id_to_vertex.begin(), landmark_id_to_vertex.end(),
      std::back_inserter(result.optimized_positions), [](auto&& item) {
        return OptimizedLandmarkPosition{item.first, item.second->estimate()};
      });

  return result;
}

g2o::VertexSE3Expmap* G2OPointBackend::CreatePoseVertex(
    const factor_graph::PoseNode& pose_node) const {
  auto vertex = new g2o::VertexSE3Expmap();
  auto pose = pose_node.GetPose();
  Eigen::Matrix3d rotation_matrix = pose.rotation();
  Eigen::Vector3d translation_vector = pose.translation();
  vertex->setEstimate(g2o::SE3Quat(rotation_matrix, translation_vector));
  vertex->setFixed(pose_node.GetID() == 0);

  return vertex;
}

g2o::VertexPointXYZ* G2OPointBackend::CreateLandmarkVertex(
    const factor_graph::LandmarkNode& landmark_node) const {
  auto vertex = new g2o::VertexPointXYZ();
  Eigen::Vector3d landmark_pos = landmark_node.GetPosition();
  vertex->setEstimate(landmark_pos);
  vertex->setMarginalized(true);
  vertex->setFixed(false);

  return vertex;
}

g2o::EdgeStereoSE3ProjectXYZ* G2OPointBackend::CreateEdge(
    g2o::OptimizableGraph::Vertex* landmark_vertex,
    g2o::OptimizableGraph::Vertex* pose_vertex,
    Eigen::Vector3d observation) const {
  auto edge = new g2o::EdgeStereoSE3ProjectXYZ();
  edge->setVertex(0, landmark_vertex);
  edge->setVertex(1, pose_vertex);
  edge->setMeasurement(observation);
  edge->setInformation(Eigen::Matrix3d::Identity());
  edge->setRobustKernel(new g2o::RobustKernelHuber{});
  edge->fx = fx_;
  edge->fy = fy_;
  edge->cx = cx_;
  edge->cy = cy_;
  edge->bf = bf_;

  return edge;
}

std::unique_ptr<g2o::SparseOptimizer> G2OPointBackend::CreateOptimizer() const {
  auto linear_solver = std::make_unique<LinearSolverType>();
  auto block_solver =
      std::make_unique<BlockSolverType>(std::move(linear_solver));
  auto solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
  auto optimizer = std::make_unique<g2o::SparseOptimizer>();
  optimizer->setAlgorithm(solver);

  return optimizer;
}

}  // namespace prime_slam::backend
