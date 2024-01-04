cmake_minimum_required(VERSION 2.8)

project(momentum)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(
    ${CERES_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})



add_library(circleGridFinder src/CCircleGridFinder.cpp)
add_library(momentumTracker src/CMomentumTracker.cpp)
add_library(lieAlgebra src/CLieAlgebra.cpp)
add_library(calibrator src/CCalibrator.cpp)
add_library(targetDetector src/CTargetDetector.cpp)

target_link_libraries(circleGridFinder
    ${OpenCV_LIBRARIES}
)
target_link_libraries(calibrator
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    momentumTracker
    lieAlgebra
)
target_link_libraries(targetDetector
    ${OpenCV_LIBRARIES}
    circleGridFinder
)

add_executable(main.out src/main.cpp)

target_link_libraries(main.out
    ${OpenCV_LIBRARIES}
    calibrator
    targetDetector
)

set_property(TARGET main.out PROPERTY CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)