cmake_minimum_required(VERSION 2.8)

project(discocal)

find_package(OpenCV REQUIRED)
find_package(Eigen3)


add_subdirectory(src/lib_calib)
add_subdirectory(src/lib_detect)
add_subdirectory(src/lib_utils)

add_executable(mono.out src/mono.cpp)
# target_include_directories(main.out ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(mono.out
    ${OpenCV_LIBRARIES}
    targetDetector
    calibrator
    utils
)
set_property(TARGET mono.out PROPERTY CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)