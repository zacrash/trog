# Trog
Trog is a pilot mobile battery energy storage system that works in integration with the grid to deliver on-demand energy.  

# Development Branch
This branch implements new features on Trog. Currently we are developing...
    
    Speeding up Control node response time

ToDo List
========
#### Miscellaneous
* Document various tasks

#### Software
* [ ] Speed up control node response time
  * There is literall a command "?C 1"
* [ ] Calibrate velocity and motor controller commands
* [ ] Get GPS readings
* [ ] Add unit tests

#### Hardware
* [ ] Establish local wifi network to avoid continually dropping connection
* [ ] Connect IMU to Jetson via serial port (using Serial1 not Serial_HARDWARE)


User Guide
===
## Get robot up and running
    $ roslaunch trog_bringup bringup.launch create_map:=<boolean> move_base:=<boolean>
    
## Teleop
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/trog_velocity_controller/cmd_vel
