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

#include "prime_slam/data/dataset_factory.h"
#include "prime_slam/data/rgbd_dataset.h"
#include "prime_slam/data/datasets/data_format.h"
#include "prime_slam/observation/description/descriptor_concept.h"
#include "prime_slam/observation/description/line/lbd.h"
#include "prime_slam/observation/description/point/orb_descriptor.h"
#include "prime_slam/observation/detection/detector_concept.h"
#include "prime_slam/observation/detection/line/lsd.h"
#include "prime_slam/observation/detection/point/orb.h"
#include "prime_slam/observation/filter/filter_chain.h"
#include "prime_slam/observation/filter/keyobject_filter_concept.h"
#include "prime_slam/observation/filter/point_fov_filter.h"
#include "prime_slam/observation/filter/point_negative_depth_filter.h"
#include "prime_slam/observation/keyobject_concept.h"
#include "prime_slam/observation/keypoint.h"
#include "prime_slam/observation/matching/match.h"
#include "prime_slam/observation/matching/matcher_concept.h"
#include "prime_slam/observation/matching/opencv_matcher.h"
#include "prime_slam/observation/observation_creator.h"
#include "prime_slam/observation/observation_creator_concept.h"
#include "prime_slam/observation/opencv_keyline.h"
#include "prime_slam/observation/opencv_keypoint.h"
#include "prime_slam/pipelines.h"
#include "prime_slam/sensor/rgbd.h"
#include "prime_slam/slam/backend/backend_concept.h"
#include "prime_slam/slam/backend/backend_g2o.h"
#include "prime_slam/slam/data_association.h"
#include "prime_slam/slam/frame/frame.h"
#include "prime_slam/slam/frame/keyframe_selection/every_nth_keyframe_selector.h"
#include "prime_slam/slam/frame/keyframe_selection/keyframe_selector_concept.h"
#include "prime_slam/slam/frontend/frontend_concept.h"
#include "prime_slam/slam/frontend/tracking_frontend.h"
#include "prime_slam/slam/mapping/map.h"
#include "prime_slam/slam/mapping/map_concept.h"
#include "prime_slam/slam/mapping/mapper.h"
#include "prime_slam/slam/mapping/mapper_concept.h"
#include "prime_slam/slam/slam.h"
#include "prime_slam/slam/tracking/tracker.h"
#include "prime_slam/slam/tracking/tracker_concept.h"
#include "prime_slam/visualization/visualize_map.h"
#include "prime_slam/metrics.h"
