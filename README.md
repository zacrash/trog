# Trog
Trog is a pilot mobile battery energy storage system that works in integration with the grid to deliver on-demand energy.  

ToDo List
========

#### Software
* [ ] Figure out why control node has slowed
* [ ] Configure move_base parameters
* [ ] Get GPS readings

#### Hardware
* [ ] Get monitor 
* [ ] Connect IMU to Jetson via serial port (using Serial1 not Serial_HARDWARE)

## Milestones
* [X] Teleoperation
* [ ] Indoor waypoint following
* [ ] Outdoor waypoint following
* [ ] Outdoor path planning and navigation

User Guide
===
    # Get robot up and running
    $ roslaunch trog_bringup bringup.launch
    
    ------------------------------------------

    # Autonomous navigation within a known map
    $ rosrun trog_2dnav known_map.launch

    OR

    # Teleop
    $  rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/trog_velocity_controller/cmd_vel
    


