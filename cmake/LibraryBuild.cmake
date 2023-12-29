set(LIBRARY_HEADER_FILES
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/slam.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/tracking/pose_estimation/pose_estimator_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/tracking/pose_estimation/rgbd_point_pose_estimator.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/data_association.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/tracking/tracker.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/tracking/tracker_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/tracking/tracking_result.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/mapping/landmark.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/mapping/map.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/mapping/map_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/mapping/mapper.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/mapping/mapper_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/graph/factor_graph.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/graph/factor/stereo_observation_factor.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/graph/node/landmark_node.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/graph/node/pose_node.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/frontend/frontend_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/frontend/tracking_frontend.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/frame/frame.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/frame/keyframe_selection/keyframe_selector_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/frame/keyframe_selection/every_nth_keyframe_selector.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/backend/backend_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/backend/backend_g2o.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/backend/backend_result.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/backend/optimized_landmark_position.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/backend/optimized_pose.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/sensor/rgbd.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/projection/point_projector.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/projection/projector.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/description/point/detail/opencv_point_descriptor.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/description/point/orb_descriptor.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/detection/point/detail/opencv_point_detector.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/detection/point/orb.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/filter/filter_chain.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/filter/keyobject_filter_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/filter/point_fov_filter.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/filter/point_negative_depth_filter.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/matching/match.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/matching/matcher_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/matching/opencv_matcher.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/traits/observation_tag.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/keyobject_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/observation_batch.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/observation_creator.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/opencv_keypoint.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/datasets/icl_nuim_dataset.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/datasets/icl_nuim_tum_format_dataset.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/datasets/rgbd_dataset_interface.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/datasets/tum_rgbd_dataset.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/datasets/tum_rgbd_dataset_base.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/observation_creator_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/detection/line/lsd.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/opencv_keyline.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/description/line/lbd.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/projection/line_projector.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/observation.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/projection/projector_concept.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/tracking/pose_estimation/rgbd_pose_estimator.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/keypoint.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/metrics.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/pipelines.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/visualization/visualize_map.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/datasets/data_format.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/dataset_factory.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/rgbd_dataset.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/prime_slam.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/datasets/default_parameters.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/data/dataset_iterator.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/observation/traits/observation_traits.h
        ${PROJECT_SOURCE_DIR}/include/prime_slam/slam/mapping/visible_landmarks_mask.h
)

set(LIBRARY_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/prime_slam/data/datasets/icl_nuim_dataset.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/data/datasets/io_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/data/datasets/tum_rgbd_dataset.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/observation/description/opencv_point_descriptor.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/observation/detection/point/opencv_point_detector.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/observation/matching/opencv_matcher.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/slam/backend/backend_g2o.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/observation/description/lbd.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/observation/detection/line/lsd.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/data/datasets/icl_nuim_tum_format_dataset.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/data/datasets/tum_rgbd_dataset_base.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/data/dataset_factory.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/observation/description/extract_cv_descriptors.h
        ${PROJECT_SOURCE_DIR}/src/prime_slam/observation/description/extract_cv_descriptors.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/metrics.cpp
        ${PROJECT_SOURCE_DIR}/src/prime_slam/data/datasets/io_utils.h
)

# Target
add_library(${LIBRARY_NAME}
        ${LIBRARY_SOURCE_FILES}
        ${LIBRARY_HEADER_FILES}
)

# Alias:
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# C++ std
target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_${CMAKE_CXX_STANDARD})

# Add definitions for targets
# Values:
#   - Debug  : -DPRIME_SLAM_DEBUG=1
#   - Release: -DPRIME_SLAM_DEBUG=0
#   - others : -DPRIME_SLAM_DEBUG=0
target_compile_definitions(${LIBRARY_NAME} PUBLIC
        "${PROJECT_NAME_UPPERCASE}_DEBUG=$<CONFIG:Debug>")

target_include_directories(
        ${LIBRARY_NAME} PUBLIC
        "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${GENERATED_HEADERS_DIR}>"
        "$<INSTALL_INTERFACE:.>"
)

find_package(Eigen3 3.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV 4.6 PATHS ${OPENCV_INSTALL_DIR} REQUIRED)

include(FetchContent)

message(STATUS "FetchContent: fmt")
FetchContent_Declare(fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG 10.1.1
)
FetchContent_MakeAvailable(fmt)

message(STATUS "FetchContent: g2o")
FetchContent_Declare(g2o
        GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o
        GIT_TAG 20230806_git
)
FetchContent_MakeAvailable(g2o)

if (${PROJECT_NAME_UPPERCASE}_BUILD_TESTS)
    message(STATUS "FetchContent: googletest")
    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.14.0
    )
    FetchContent_MakeAvailable(googletest)
endif ()

target_include_directories(${LIBRARY_NAME}
        PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        PRIVATE
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME}
        PUBLIC
        fmt::fmt
        Eigen3::Eigen
        Boost::program_options
        g2o_core
        g2o_stuff
        g2o_types_sba
        g2o_solver_csparse
        g2o_solver_dense
        ${OpenCV_LIBS}
)

if (${PROJECT_NAME_UPPERCASE}_BUILD_VISUALIZER)
    find_package(Open3D)
    target_link_libraries(${LIBRARY_NAME}
            PUBLIC
            Open3D::Open3D
    )
    target_compile_definitions(
            ${LIBRARY_NAME} PUBLIC
            ${PROJECT_NAME_UPPERCASE}_BUILD_VISUALIZER
    )
endif ()

# Targets:
install(
        TARGETS "${LIBRARY_NAME}"
        EXPORT "${TARGETS_EXPORT_NAME}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

# Headers:
foreach (file ${LIBRARY_HEADER_FILES})
    get_filename_component(dir ${file} DIRECTORY)
    install(FILES ${file} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_FOLDER}/${dir})
endforeach ()


# Headers:
install(
        FILES "${GENERATED_HEADERS_DIR}/${LIBRARY_FOLDER}/version.h"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_FOLDER}"
)

# Config
install(
        FILES "${PROJECT_CONFIG_FILE}"
        "${VERSION_CONFIG_FILE}"
        DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Config
install(
        EXPORT "${TARGETS_EXPORT_NAME}"
        FILE "${PROJECT_NAME}Targets.cmake"
        DESTINATION "${CONFIG_INSTALL_DIR}"
        NAMESPACE "${PROJECT_NAME}::"
)
