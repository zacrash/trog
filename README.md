# miniature-parakeet
For troggie

## Tonight's TODOs
### Getting the map
- First we need to create a map. ```$ roslaunch trog_mapping slam.launch``` Makes map with teleop.
- Once done run: ```rosrun map_server map_saver -f trog_mapping/maps/lab_map"``` to save map. 
------------
### Running the navigation stack
- After map is made, run ```$ roslaunch trog_2dnav known_map.launch``` to run navigation stack in a known map. Goal is from lab to outside and back.
- Call ```rosservice call move_base/global_localization``` at beginning
- Then launch RViz. ```rosrun rviz rviz```