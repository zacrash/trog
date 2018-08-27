
Overview
========
This folder contains a slightly modified fo the [Velodyne ROS driver repository](https://github.com/ros-drivers/velodyne). The current repository does not publish GPS data. The velodyne_driver code was modified to capture the position packets, grab the GPS NMEA setence and publish it as a ROS topic.