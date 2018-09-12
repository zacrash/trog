# miniature-parakeet
For troggie ~.~

## User Guide
    # Get robot up and running
    $ roslaunch trog_bringup bringup.launch
    
    # Run teleop to show mobility
    $  rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/trog_velocity_controller/cmd_vel



    # Now show move base
    $ rosrun trog_2dnav known_map.launch

    # Reset AMCL
    $ rosservice call /global_localization

    # Send Nav goal via RViz
    
## DEMO
    # Run this
        rostopic pub /path_ready std_msgs/Empty -1

    # Buffer move base goals using RViz
    # Use RViz’s "2D Pose Estimate" tool to create waypoints with mouse clicks.

    # Follow waypoints
        rostopic pub /path_ready std_msgs/Empty -1


## Tonight's TODOs
### Map entire 4th floor of engineering

### Running the navigation stack
- After map is made, run ```$ roslaunch trog_2dnav known_map.launch``` to run navigation stack in a known map. Goal is from lab to outside and back.
- Call ```rosservice call move_base/global_localization``` at beginning
- Then launch RViz. ```rosrun rviz rviz```
