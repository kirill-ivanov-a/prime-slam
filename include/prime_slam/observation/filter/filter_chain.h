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
#include <utility>
#include <vector>

#include "prime_slam/observation/filter/keyobject_filter_concept.h"
#include "prime_slam/observation/filter/point_negative_depth_filter.h"

namespace prime_slam::observation {

template <Keyobject KeyobjectType>
class FilterChain {
 private:
  class IKeyobjectFilter;

  template <KeyobjectsFilter<KeyobjectType> Filter>
  class ChainedKeyobjectFilter;

 public:
  template <KeyobjectsFilter<KeyobjectType>... Filters>
  FilterChain(Filters... filters);

  std::vector<KeyobjectType> Apply(std::vector<KeyobjectType> keyobjects,
                                   const sensor::RGBDImage& sensor_data) const;

 private:
  std::vector<std::unique_ptr<IKeyobjectFilter>> filters_;
};

// IMPLEMENTATION

template <Keyobject KeyobjectType>
class FilterChain<KeyobjectType>::IKeyobjectFilter {
 public:
  virtual ~IKeyobjectFilter() {}

  virtual std::vector<KeyobjectType> Apply(
      std::vector<KeyobjectType> keyobjects,
      const sensor::RGBDImage& sensor_data) const = 0;
};

template <Keyobject KeyobjectType>
template <KeyobjectsFilter<KeyobjectType> Filter>
class FilterChain<KeyobjectType>::ChainedKeyobjectFilter final
    : public IKeyobjectFilter {
 public:
  ChainedKeyobjectFilter(Filter filter) : self_(std::move(filter)) {}

  std::vector<KeyobjectType> Apply(
      std::vector<KeyobjectType> keyobjects,
      const sensor::RGBDImage& sensor_data) const override {
    return self_.Apply(std::move(keyobjects), sensor_data);
  }

 private:
  Filter self_;
};

template <Keyobject KeyobjectType>
template <KeyobjectsFilter<KeyobjectType>... Filters>
inline FilterChain<KeyobjectType>::FilterChain(Filters... filters) {
  (filters_.emplace_back(
       std::make_unique<ChainedKeyobjectFilter<Filters>>(std::move(filters))),
   ...);
}

template <Keyobject KeyobjectType>
inline std::vector<KeyobjectType> FilterChain<KeyobjectType>::Apply(
    std::vector<KeyobjectType> keyobjects,
    const sensor::RGBDImage& sensor_data) const {
  for (auto&& filter : filters_) {
    keyobjects = filter->Apply(std::move(keyobjects), sensor_data);
  }
  keyobjects.shrink_to_fit();
  return keyobjects;
}

}  // namespace prime_slam::observation
