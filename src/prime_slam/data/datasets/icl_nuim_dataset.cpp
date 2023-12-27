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

#include "prime_slam/data/datasets/icl_nuim_dataset.h"

#include <fstream>

#include <fmt/core.h>
#include <boost/algorithm/string.hpp>
#include <Eigen/Eigen>
#include <opencv2/imgcodecs.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

namespace prime_slam {

namespace {

struct CameraParams {
  Eigen::Vector3d position;
  Eigen::Vector3d direction;
  Eigen::Vector3d up;
  Eigen::Vector3d lookat;
  Eigen::Vector3d sky;
  Eigen::Vector3d right;
  Eigen::Vector3d fpoint;
  double angle;
};

class CameraParamsParser {
 public:
  CameraParams Parse(const fs::path& camera_params_path) {
    if (!fs::exists(camera_params_path)) {
      throw fs::filesystem_error("File does not exist.", camera_params_path,
                                 std::error_code());
    }
    std::ifstream file(camera_params_path);
    CameraParams params;
    if (file.is_open()) {
      std::unordered_map<std::string, std::vector<double>> data;
      std::string line;
      while (std::getline(file, line)) {
        auto [key, value] = ParseKeyValuePair(line);

        if (IsVector(value)) {
          TrimVector(value);
        }
        data[key] = ParseVector(value);
      }

      params.position = ConvertValuesToEigenVector3d(data["cam_pos"]);
      params.direction = ConvertValuesToEigenVector3d(data["cam_dir"]);
      params.up = ConvertValuesToEigenVector3d(data["cam_up"]);
      params.lookat = ConvertValuesToEigenVector3d(data["cam_lookat"]);
      params.sky = ConvertValuesToEigenVector3d(data["cam_sky"]);
      params.right = ConvertValuesToEigenVector3d(data["cam_right"]);
      params.fpoint = ConvertValuesToEigenVector3d(data["cam_fpoint"]);
      params.angle = data["cam_angle"].front();

    } else {
      auto msg =
          fmt::format("Failed to open file: {}", camera_params_path.string());
      throw std::ifstream::failure(msg);
    }

    return params;
  }

 private:
  Eigen::Vector3d ConvertValuesToEigenVector3d(
      const std::vector<double>& values) {
    assert(values.size() == 3);
    auto x = values[0];
    auto y = values[1];
    auto z = values[2];

    return {x, y, z};
  }

  void TrimVector(std::string& input, size_t start_offset = 1,
                  size_t end_offset = 3) {
    input = input.substr(start_offset, input.length() - end_offset);
  }
  bool IsVector(const std::string& input, char vector_start_symbol = '[') {
    return input.front() == vector_start_symbol;
  }

  std::vector<double> ParseVector(const std::string& value) {
    std::vector<double> numbers;
    boost::tokenizer<boost::escaped_list_separator<char>> tokenizer(value);

    for (const auto& token : tokenizer) {
      numbers.push_back(boost::lexical_cast<double>(boost::trim_copy(token)));
    }

    return numbers;
  }

  std::pair<std::string, std::string> ParseKeyValuePair(
      std::string& input, std::string equal_sign = "=") {
    // Remove statements delimiter
    if (input.ends_with(';')) {
      input.erase(input.size() - 1);
    }
    auto tokens = SplitString(input, equal_sign);
    if (tokens.size() != 2) {
      auto msg = fmt::format(
          "Unexpected parameter format: '{}'. Expected: <key> = <value>",
          input);
      throw std::invalid_argument{msg};
    }
    for (auto& token : tokens) {
      boost::trim(token);
    }
    auto&& key = tokens[0];
    if (key.empty()) {
      auto msg = fmt::format("Empty key: '{}'", input);
      throw std::invalid_argument{msg};
    }
    auto&& value_string = tokens[1];
    if (value_string.empty()) {
      auto msg = fmt::format("Empty value: '{}'", input);
      throw std::invalid_argument{msg};
    }

    return std::make_pair(key, value_string);
  }

  std::vector<std::string> SplitString(const std::string& input,
                                       const std::string& delimiter) {
    std::vector<std::string> tokens;
    boost::split(tokens, input, boost::is_any_of(delimiter));
    return tokens;
  }
};

Eigen::VectorXd ReadEuclideanDistancesFile(const fs::path& path) {
  std::ifstream file(path);
  Eigen::VectorXd euclidean_distances;

  if (file.is_open()) {
    std::vector<double> coefficients;
    std::copy(std::istream_iterator<double>(file),
              std::istream_iterator<double>(),
              std::back_inserter(coefficients));
    euclidean_distances =
        Eigen::Map<Eigen::VectorXd>(coefficients.data(), coefficients.size());
  } else {
    auto msg = fmt::format("Failed to open depth file: {}", path.string());
    throw std::ifstream::failure(msg);
  }

  return euclidean_distances;
}

Eigen::MatrixXd ConvertEuclideanDistancesToDepthMap(
    const Eigen::VectorXd& euclidean_distances, size_t depth_map_height,
    size_t depth_map_width, const Eigen::Projective3d& intrinsics) {
  auto euclidean_dists_map =
      euclidean_distances.reshaped(depth_map_width, depth_map_height).eval();
  auto depth_map =
      Eigen::MatrixXd::Zero(depth_map_height, depth_map_width).eval();

  auto fx = intrinsics(0, 0);
  auto fy = intrinsics(1, 1);
  auto cx = intrinsics(0, 2);
  auto cy = intrinsics(1, 2);

  for (auto y = 0ul; y != depth_map_height; ++y) {
    for (auto x = 0ul; x != depth_map_width; ++x) {
      auto x_direction = (x - cx) / fx;
      auto y_direction = (y - cy) / fy;
      double depth =
          euclidean_dists_map(x, y) /
          std::sqrt(x_direction * x_direction + y_direction * y_direction + 1);
      depth_map(y, x) = depth;
    }
  }

  return depth_map;
}

Eigen::Projective3d CreatePose(const CameraParams& params) {
  auto&& direction = params.direction;
  auto&& up = params.up;
  auto z = direction.normalized();
  auto x = up.cross(z).normalized();
  auto y = z.cross(x);
  auto rotation = Eigen::Matrix3d{};
  auto translation = params.position;
  rotation.col(0) = x;
  rotation.col(1) = y;
  rotation.col(2) = z;

  auto transform_matrix = Eigen::Projective3d::Identity();
  transform_matrix.rotate(rotation);
  transform_matrix.translate(translation);

  return transform_matrix;
}

}  // namespace

ICLNUIMDataset::ICLNUIMDataset(const fs::path& data_directory_path,
                               const std::string& rgb_extension,
                               const std::string& depth_extension,
                               const std::string& camera_parameters_extension,
                               const Eigen::Projective3d& intrinsics)
    : intrinsics_(intrinsics) {
  if (!exists(data_directory_path)) {
    throw fs::filesystem_error("Directory does not exist.", data_directory_path,
                               std::error_code());
  }
  for (auto&& entry : fs::directory_iterator(data_directory_path)) {
    if (entry.is_regular_file()) {
      auto&& extension = entry.path().extension().string();
      if (extension == rgb_extension) {
        rgb_images_paths_.push_back(entry);
      } else if (extension == depth_extension) {
        depths_paths_.push_back(entry);
      } else if (extension == camera_parameters_extension) {
        camera_parameters_paths_.push_back(entry);
      }
      // if there is a file with a different extension, then ignore it
    }
  }
  auto path_less = [](const std::filesystem::directory_entry& lhs,
                      const std::filesystem::directory_entry& rhs) {
    return lhs.path().stem() < rhs.path().stem();
  };
  std::sort(rgb_images_paths_.begin(), rgb_images_paths_.end(), path_less);
  std::sort(depths_paths_.begin(), depths_paths_.end(), path_less);
  std::sort(camera_parameters_paths_.begin(), camera_parameters_paths_.end(),
            path_less);
  gt_poses_ = CreateGroundTruthPoses();
}

size_t ICLNUIMDataset::Size() const noexcept {
  return rgb_images_paths_.size();
}

const std::vector<Eigen::Projective3d>& ICLNUIMDataset::GetGroundTruthPoses()
    const& noexcept {
  return gt_poses_;
}

const Eigen::Projective3d& ICLNUIMDataset::GetIntrinsics() const& noexcept {
  return intrinsics_;
}

sensor::RGBDImage ICLNUIMDataset::operator[](size_t index) const {
  auto rgb_img = cv::imread(rgb_images_paths_[index].path());
  auto depth_map = ConvertEuclideanDistancesToDepthMap(
      ReadEuclideanDistancesFile(depths_paths_[index]), rgb_img.rows,
      rgb_img.cols, intrinsics_);
  return sensor::RGBDImage{std::move(rgb_img), std::move(depth_map)};
}

std::vector<Eigen::Projective3d> ICLNUIMDataset::CreateGroundTruthPoses()
    const {
  auto poses = std::vector<Eigen::Projective3d>();
  poses.reserve(camera_parameters_paths_.size());
  std::transform(
      camera_parameters_paths_.begin(), camera_parameters_paths_.end(),
      std::back_inserter(poses), [](auto&& camera_params_path) {
        return CreatePose(CameraParamsParser{}.Parse(camera_params_path));
      });

  return poses;
}

}  // namespace prime_slam
