cmake_minimum_required(VERSION 2.8)
# set(CMAKE_CXX_STANDARD 20)
find_package(OpenCV REQUIRED)
find_package(Eigen3)
find_package(OpenMP)


add_library(targetDetector CTargetDetector.cpp CCircleGridFinder.cpp)
if(OpenMP_CXX_FOUND)
    target_link_libraries(targetDetector PUBLIC OpenMP::OpenMP_CXX)
endif()
target_include_directories(targetDetector PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})