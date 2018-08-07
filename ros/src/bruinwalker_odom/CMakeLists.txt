cmake_minimum_required(VERSION 2.8.3)
project(bruinwalker_odom)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bruinwalker_odom
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(test_odom_node src/test_odom_node.cpp)

target_link_libraries(test_odom_node
   ${catkin_LIBRARIES}
 )