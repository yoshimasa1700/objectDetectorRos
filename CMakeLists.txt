cmake_minimum_required(VERSION 2.8.3)
project(objectDetector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  image_transport
)
find_package(OpenCV)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/local/include)

add_executable(objectDetector src/objectDetector.cpp)
target_link_libraries(objectDetector ${catkin_LIBRARIES} HFMD_core)

