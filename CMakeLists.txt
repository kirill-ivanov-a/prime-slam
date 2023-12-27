cmake_minimum_required(VERSION 3.16)
project(prime_slam
        DESCRIPTION "SLAM system with flexible module configuration"
        LANGUAGES CXX
        VERSION 1.0.0)

string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWERCASE)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)

set(LIBRARY_NAME ${PROJECT_NAME_LOWERCASE})
# Library folder name (by default is the project name in lowercase)
set(LIBRARY_FOLDER ${PROJECT_NAME_LOWERCASE})
# Make sure different configurations don't collide
set(CMAKE_DEBUG_POSTFIX "d")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(${LIBRARY_NAME}_BUILD_TESTS "Build ${LIBRARY_NAME} tests" ON)
option(${LIBRARY_NAME}_BUILD_EXAMPLES "Build ${LIBRARY_NAME} examples" ON)
option(${LIBRARY_NAME}_INSTALL "Generate target for installing ${LIBRARY_NAME}" ${PROJECT_IS_TOP_LEVEL})
option(${LIBRARY_NAME}_HIDE_INTERNAL_SYMBOLS "Build ${LIBRARY_NAME} with internal symbols hidden in shared libraries.")
option(BUILD_SHARED_LIBS "Build ${LIBRARY_NAME} as a shared library." OFF)

message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")

include(${PROJECT_SOURCE_DIR}/cmake/SetEnv.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LibraryBuild.cmake)

if (${LIBRARY_NAME}_BUILD_TESTS)
    add_subdirectory(
            ${PROJECT_SOURCE_DIR}/tests
    )
endif ()

if (${LIBRARY_NAME}_BUILD_EXAMPLES)

endif ()
add_subdirectory(
        ${PROJECT_SOURCE_DIR}/examples
)
