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

#include <boost/program_options.hpp>
#include <boost/timer/progress_display.hpp>
#include <fmt/format.h>
#include <prime_slam/prime_slam.h>

#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;
namespace po = boost::program_options;
namespace ps = prime_slam;

ps::DataFormat StringToDataFormat(const std::string &format) {
  if (format == "tum") {
    return ps::DataFormat::TUM;
  } else if (format == "icl") {
    return ps::DataFormat::ICL;
  } else if (format == "icl_tum") {
    return ps::DataFormat::ICL_TUM;
  }
  throw std::invalid_argument(fmt::format("Invalid data format: {}", format));
}

void PrintHelpMessage(const std::string &program_name,
                      const po::options_description &optionals) {
  std::cout << "Usage: " << program_name << " [options] <DATASET_PATH>\n";
  std::cout << optionals << '\n';
}

int main(int argc, char *argv[]) {
  constexpr const auto kDataPathParam = "data-path";
  constexpr const auto kDataFormatParam = "data-format";
  constexpr const auto kKeyframesStepParam = "keyframes-step";

  po::options_description options_description("Allowed options");
  options_description.add_options()("help", "Produce help message")(
      kDataPathParam, po::value<std::string>(), "Path to dataset")(
      kDataFormatParam, po::value<std::string>()->default_value("icl_tum"),
      "Format of dataset (tum, icl, icl_tum)")(
      kKeyframesStepParam, po::value<size_t>()->default_value(10),
      "Step between keyframes");

  po::positional_options_description positional_options;
  positional_options.add(kDataPathParam, 1);

  po::variables_map parsed_variables;
  po::store(po::command_line_parser(argc, argv)
                .options(options_description)
                .positional(positional_options)
                .run(),
            parsed_variables);
  po::notify(parsed_variables);
  if (parsed_variables.count("help")) {
    PrintHelpMessage(argv[0], options_description);
  } else if (parsed_variables.count(kKeyframesStepParam) == 0) {
    std::cout << fmt::format("Missing positional argument: {}",
                             kKeyframesStepParam);
    PrintHelpMessage(argv[0], options_description);
  } else {
    auto base_path =
        fs::path(parsed_variables[kDataPathParam].as<std::string>());
    auto data_format = StringToDataFormat(
        parsed_variables[kDataFormatParam].as<std::string>());
    auto dataset = prime_slam::DatasetFactory::Create(data_format, base_path);
    auto &&gt_poses = dataset.GetGroundTruthPoses();
    auto step = parsed_variables[kKeyframesStepParam].as<std::size_t>();
    auto slam = prime_slam::CreateDefaultORBPipeline(dataset.GetIntrinsics(),
                                                     gt_poses.front(), step);

    boost::timer::progress_display progress_bar{dataset.Size()};
    for (auto &&data : dataset) {
      ++progress_bar;
      slam.ProcessSensorData(std::move(data));
    }
#ifdef PRIME_SLAM_BUILD_VISUALIZER
    auto &&map = slam.GetMap();
    prime_slam::visualization::VisualizeMap(map);
#endif
  }

  return 0;
}
