include(GoogleTest)

function(prime_slam_test_target target)
    set(data OPTIONAL DATA_NEEDED)
    cmake_parse_arguments(PRIME_SLAM_TEST "${data}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(
            ${target}
            ${target}.cpp
    )
    message(STATUS "Add test ${target}")
    target_link_libraries(${target} PRIVATE ${LIBRARY_NAME})
    target_link_libraries(${target} PRIVATE GTest::gtest_main)
    gtest_discover_tests(${target})
    if (PRIME_SLAM_TEST_DATA_NEEDED)
        target_compile_definitions(${target} PUBLIC TEST_DATA_DIR="${TEST_DATA_DIR}")
    endif ()
endfunction()
