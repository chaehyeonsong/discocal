find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS 
    common 
    io 
    visualization 
    filters 
    segmentation       # EuclideanClusterExtraction
    search             # KdTree
)
add_library(lidarModule
    CLidarextrinsic.cpp
)
target_include_directories(lidarModule PUBLIC
    ${OpenCV_INCLUDE_DIRS}    
    ${EIGEN3_INCLUDE_DIRS}    
    ${PCL_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(lidarModule
    ${OpenCV_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
)


