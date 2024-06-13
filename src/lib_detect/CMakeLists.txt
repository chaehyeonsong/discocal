cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)
find_package(Eigen3)

add_library(targetDetector CTargetDetector.cpp CCircleGridFinder.cpp)
target_include_directories(targetDetector PUBLIC ${CMAKE_SOURCE_DIR}/src/includes ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
# target_link_libraries(calibrator
#     ${OpenCV_LIBRARIES}
#     ${CERES_LIBRARIES}
# )
