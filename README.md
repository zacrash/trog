# Trog v0.1
Trog is a pilot mobile battery energy storage system that works in integration with the grid to deliver on-demand energy.

## Status
* [X] Teleoperation
* [ ] Indoor navigation
* [ ] Outdoor navigatoin

User Guide
===
    # Get robot up and running
    $ roslaunch trog_bringup bringup.launch create_map:= <true or false> move_base:=<true or false>
    
    # Teleop
    $  rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/trog_velocity_controller/cmd_vel
   
