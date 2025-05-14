cmake_minimum_required(VERSION 2.8)

add_library(calibrator CCalibrator.cpp CMomentsTracker.cpp CLieAlgebra.cpp)
target_include_directories(calibrator PUBLIC 
    ${CMAKE_SOURCE_DIR}/include 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(calibrator
    ${CERES_LIBRARIES}
)