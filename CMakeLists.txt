cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(discocal)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
add_subdirectory(src/lib_calib)
add_subdirectory(src/lib_detect)
add_subdirectory(src/lib_lidar)
# add_subdirectory(src/lib_utils)

add_executable(mono.out src/main_mono.cpp)
target_link_libraries(mono.out
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    yaml-cpp
)
set_property(TARGET mono.out PROPERTY CXX_STANDARD 20)

add_executable(stereo.out src/main_stereo.cpp)
target_link_libraries(stereo.out
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    yaml-cpp
)
set_property(TARGET stereo.out PROPERTY CXX_STANDARD 20)

add_executable(lidar.out src/main_lidar.cpp)
target_link_libraries(lidar.out
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    lidarModule   
    yaml-cpp
)
set_property(TARGET lidar.out PROPERTY CXX_STANDARD 20)

add_subdirectory(pybind11)
pybind11_add_module(pydiscocal src/main_bind.cpp)
target_link_libraries(pydiscocal PRIVATE
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    lidarModule
    yaml-cpp
)
