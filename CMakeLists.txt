cmake_minimum_required(VERSION 2.8)

project(discocal)

find_package(OpenCV REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp REQUIRED)
# include(ExternalProject)
# include("${CMAKE_SOURCE_DIR}/vendor/rapidjson.cmake")
# include_directories(${RAPIDJSON_INCLUDE_DIR})

add_subdirectory(src/lib_calib)
add_subdirectory(src/lib_detect)
add_subdirectory(src/lib_utils)

add_executable(mono.out src/mono.cpp)
target_link_libraries(mono.out
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    utils
    yaml-cpp
)
# add_dependencies(mono.out rapidjson)
set_property(TARGET mono.out PROPERTY CXX_STANDARD 17)

add_executable(stereo.out src/stereo.cpp)
target_link_libraries(stereo.out
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    utils
    yaml-cpp
)
set_property(TARGET stereo.out PROPERTY CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD 17)