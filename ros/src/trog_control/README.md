trog_Control TODOs
=====

## About
    This node is the hardware interface for ROS Control necessary for diff_drive_controller. 
      (twist commands) -> [diff_drive_controller -> trog_control_node -> roboteq_driver] -> (to motor controller)

## trog_hardware.cc
    Implement updateJointsFromHardware()
