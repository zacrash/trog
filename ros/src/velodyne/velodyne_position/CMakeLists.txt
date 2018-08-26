cmake_minimum_required(VERSION 2.8.3)
project(velodyne_position)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  velodyne_driver
  velodyne_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp velodyne_driver velodyne_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(velodyne_position_node src/velodyne_position_node.cc)
 
 target_link_libraries(velodyne_position_node
   ${catkin_LIBRARIES}
 )

