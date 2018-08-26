cmake_minimum_required(VERSION 2.8.3)
project(velodyne_position)

find_package(catkin REQUIRED)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
# ${catkin_INCLUDE_DIRS}
)

add_executable(velodyne_to_nmea_node src/rawPositionData_node.cpp)
