cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)
find_package(Eigen3)

add_library(utils CImagehandler.cpp)
target_include_directories(utils PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
