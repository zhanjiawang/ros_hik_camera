cmake_minimum_required( VERSION 2.8.3 )
project(ros_hik_camera)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(DRIVER_INCLUDE /opt/MVS/include)
set(DRIVER_LIBRARY_PATH /opt/MVS/lib/aarch64)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
)

find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${DRIVER_INCLUDE}
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

file(GLOB_RECURSE DRIVER_LIBRARY ${DRIVER_LIBRARY_PATH}/*.so)

add_executable(ros_hik_camera main.cpp)

target_link_libraries(ros_hik_camera  
    pthread
    ${catkin_LIBRARIES} 
    ${DRIVER_LIBRARY} 
    ${OpenCV_LIBS}
    ${cv_bridge_LIBRARIES}
)
