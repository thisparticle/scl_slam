cmake_minimum_required(VERSION 2.8.3)
project(dlc_slam)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  distributed_mapper_catkin
  gtsam_catkin
  pcl_catkin
  tf
  roscpp
  # rospy
  # cv_bridge
  # pcl library
  # pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(libnabo)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  loop_info.msg
  global_descriptor.msg
)

add_service_files(
  DIRECTORY srv
  FILES 
  save_distributed_trajectory.srv
  geometric_verification.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  message_runtime
)

# include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# link directories
link_directories(
  include
  ${OpenCV_LIBRARY_DIRS}
  ${libnabo_LIBRARY_DIRS}
)

###########
## Build ##
###########
# Distributed Mapping
add_executable(${PROJECT_NAME}_distributedMapping src/distributedMapping.cpp)
target_link_libraries(${PROJECT_NAME}_distributedMapping ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${libnabo_LIBRARIES} global_map_solver graph_utils robot_measurements fast_max-clique_finder pairwise_consistency distributed_mapper distributed_pcm)
