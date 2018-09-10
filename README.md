# miniature-parakeet
For troggie

## Tonight's TODOs
### Getting the map
- First we need to create a map. ```$ roslaunch trog_mapping slam.launch``` Makes map with teleop.
- Once done run: ```rostopic pub syscommand std_msgs/String "savegeotiff"``` to save map. 
------------
### Running the navigation stack
- After map is made, run ```$ roslaunch trog_2dnav known_map.launch``` to run navigation stack in a known map. Goal is from lab to outside and back.
