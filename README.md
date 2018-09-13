# Trog
Trog is a pilot mobile battery energy storage system that works in integration with the grid to deliver on-demand energy.  

ToDo List
========

#### Software
* [ ] Install AMCL and Map Server and remove folder
* [ ] Get GPS readings
* [ ] Implement trog_odom
* [ ] Configure move_base parameters
* [ ] Make URDF to be correct

#### Hardware
* [ ] Wire encoders
* [ ] Connect IMU to Jetson via serial port
* [ ] Replace exisiting sprockets with D-shaft

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
    


