<div align="center">
  <img src="https://raw.githubusercontent.com/kirill-ivanov-a/prime-slam/main/assets/logo.png">
</div>

# prime-slam
Prime SLAM is a Simultaneous Localization and Mapping (SLAM) system with flexible module configuration. 
The idea to create this system arose after using many other SLAM systems that do not provide an easy way to configure, 
for example, using a different type of landmarks is difficult. Most of the code is written using templates and concepts, 
which provides configuration flexibility
# Platforms
Currently only Linux-Based systems are supported (tested on Ubuntu 20.04).
# Installation
## Prerequisites
- git
- CMake >= 3.16
- C++ compiler (with C++20 support, for example, clang-14)
## Installation steps
1. Clone this repository:
```c++
git clone 
```
2. Install [OpenCV](https://github.com/opencv/opencv) >= 4.6
3. Install [g2o](https://github.com/RainerKuemmerle/g2o)
4. Install other dependencies:
```c++
sudo apt update
sudo apt install -y  \
libeigen3-dev libboost-dev \
libboost-program-options-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 libsuitesparse-dev
```
5. Build library:
```c++
mkdir build && cd build \
cmake .. \
      -DPRIME_SLAM_BUILD_TESTS=ON \
      -DCMAKE_BUILD_TYPE=${{ env.build_type }} \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
make -j4
sudo make install
```
# Vocabulary

The terminology used within the project is described [here](docs/vocabulary.md).

# Quick example
```c++
#include <filesystem>
#include <iostream>

#include <prime_slam/prime_slam.h>

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
  constexpr const auto path = "PATH_TO_DATASET";
  auto base_path = fs::path(path);
  // Select correct data format
  auto data_format = prime_slam::DataFormat::ICL_TUM;
  auto dataset = prime_slam::DatasetFactory::Create(data_format, base_path);
  auto &&gt_poses = dataset.GetGroundTruthPoses();
  auto slam = prime_slam::CreateDefaultORBPipeline(dataset.GetIntrinsics(),
                                                   gt_poses.front());
  for (auto &&data : dataset) {
    slam.ProcessSensorData(std::move(data));
  }

  return 0;
}
```
In this example default ORB pipeline is created.

# Data format

The currently available data formats are described [here](docs/data_formats.md). However, you can add your own.

# Customizing pipeline
The system is easy to customize, for example, you can add your own detector:
```c++
class CustomPointDetector {
 public:
  using ObservationType = PointObservationTag;
  
  std::vector<Keypoint> Detect(const cv::Mat& rgb_image) const {
    // Just returns zero coordinates
    return {Keypoint{0.0, 0.0}};
  }
};

static_assert(Detector<CustomPointDetector>); // Check concept satisfaction
```
In a similar manner, you can redefine any system module.

# Structure

| Entry               | Description                                               |
|:--------------------|:----------------------------------------------------------|
| `📁 .github`        | CI/CD scripts and GitHub related files                    |
| `📁 cmake`          | Cmake scripts                                             |
| `📁 docs`           | Files with additional system description                  |
| `📁 examples`       | Examples of system use                                    |
| `📁 include`        | system include files                                      |
| `📁 src`            | system source files                                       |
| `📁 tests`          | unit tests                                                |
| `📄 .clang_format`  | code style file                                           |
| `📄 CMakeLists.txt` | CMake configuration, add as sub directory to your project |

# License
Apache License Version 2.0
