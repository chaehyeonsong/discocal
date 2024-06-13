cmake_minimum_required(VERSION 3.16)
# set(CMAKE_CXX_STANDARD 17)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3)
# include_directories(
#     ${CERES_INCLUDE_DIRS}
# )
# include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(calibrator CCalibrator.cpp CMomentsTracker.cpp CLieAlgebra.cpp)
target_include_directories(calibrator PUBLIC ${CMAKE_SOURCE_DIR}/src/includes ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(calibrator
    ${CERES_LIBRARIES}
)
