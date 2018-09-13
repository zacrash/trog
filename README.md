# Trog
Trog is a pilot mobile battery energy storage system that works in integration with the grid to deliver on-demand energy.  

## TODOs
* [ ] Wire encoders
* [ ] Connect IMU to Jetson via serial port
* [ ] Replace exisiting sprockets with D-shaft

## Milestones
* [X] Teleop working
* [ ] Indoor waypoint following
* [ ] Outdoor waypoint following
* [ ] Outdoor path planning and navigation

## User Guide
    # Get robot up and running
    $ roslaunch trog_bringup bringup.launch

    # Now autonomous navigation within a known map
    $ rosrun trog_2dnav known_map.launch

    OR

    # Run teleop to show mobility
    $  rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/trog_velocity_controller/cmd_vel
    


