cmake_minimum_required(VERSION 2.8.3)
project(roboteq_driver)

find_package(catkin REQUIRED COMPONENTS
  roboteq_msgs
  roscpp serial
  message_generation)

# Create Feedback service
add_service_files(
    FILES
    Feedback.srv
 )

generate_messages(
  DEPENDENCIES
  roboteq_driver
)

catkin_package(
  INCLUDE_DIRS include
)


# Build and install binary driver.
include_directories(include ${catkin_INCLUDE_DIRS})
add_subdirectory(mbs)
add_subdirectory(src)

add_executable(odom_forwarder src/odom_forwarder.cpp)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

