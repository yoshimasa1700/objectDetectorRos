cmake_minimum_required(VERSION 2.8.3)
project(objectDetector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  message_generation
)
find_package(OpenCV)


add_message_files(
  FILES
  Detect.msg
  Pos.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/local/include)

add_executable(objectDetector src/objectDetector.cpp)
target_link_libraries(objectDetector ${catkin_LIBRARIES} HFMD_core)
