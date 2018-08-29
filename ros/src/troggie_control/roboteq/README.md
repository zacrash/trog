roboteq
=======

This provides a ROS interface for a Roboteq motor controller. We are using a SDC2130. 

## How to use

This driver has three command modes: STOPPED, VELOCITY and POSITION. Velocity is the default. Position is for fine-grained movements or active braking on an incline. 

### For our purposes... 
- Publish velocity (rad/s) commands on "/cmd" topic
- Publish position (rad) command on "/cmd" topic
