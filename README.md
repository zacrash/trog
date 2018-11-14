# Trog
Trog is a pilot mobile battery energy storage system that works in integration with the grid to deliver on-demand energy.  

# Development Branch
This branch implements new features on Trog. Currently we are developing...
    
    Speeding up Control node response time

ToDo List
========

#### Software
* [ ] Speed up control node response time
* [ ] Get GPS readings
* [ ] Add unit tests

#### Hardware
* [ ] Connect IMU to Jetson via serial port (using Serial1 not Serial_HARDWARE)


User Guide
===
## Get robot up and running
    $ roslaunch trog_bringup bringup.launch create_map:=<boolean>
    
## Teleop
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/trog_velocity_controller/cmd_vel

## Autonomous navigation within a known map
    $ roslaunch trog_2dnav move_base.launch


