#include <boost/assign/list_of.hpp>
#include <string>
#include "trog_control/trog_hardware.h"
#include <ros/ros.h>

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
  const double WHEEL_DIAMETER = 0.254;
};

namespace trog_control
{

  /**
  * Initialize Trog hardware
  */
  TrogHardware::TrogHardware(ros::NodeHandle nh, ros::NodeHandle private_nh): nh_(nh), private_nh_(private_nh)
  {
     left_motor_pub = nh.advertise<roboteq_msgs::Command>("/left/cmd", 50); //TODO: 50?
     right_motor_pub = nh.advertise<roboteq_msgs::Command>("/right/cmd", 50); //TODO: 50?

    registerControlInterfaces();
    
    // TODO: Set up encoders
  }

  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void TrogHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

    /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void TrogHardware::writeCommandsToHardware()
  {
    double diff_speed_left = joints_[LEFT].velocity_command;
    double diff_speed_right = joints_[RIGHT].velocity_command;

    // TODO: Figure out how useful this is
    // limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    // Set up messages
    roboteq_msgs::Command cmd_left;
    cmd_left.mode = cmd_left.MODE_VELOCITY;
    cmd_left.setpoint = diff_speed_left;

    roboteq_msgs::Command cmd_right;
    cmd_right.mode = cmd_right.MODE_VELOCITY;
    cmd_right.setpoint = diff_speed_right;

    //Publish
    left_motor_pub.publish(cmd_left);
    right_motor_pub.publish(cmd_right);
  }

  void TrogHardware::updateJointsFromHardware()
  {
    ros::ServiceClient client = nh_.serviceClient<roboteq_driver::Feedback>("get_feedback");
    roboteq_driver::Feedback srv;

    for (int channel_num = 1; channel_num <= 2; channel_num++) {
      srv.request.channel = channel_num;
      if (client.call(srv))
        joints_[channel_num].velocity = srv.response.measuredVelocity;
      else
        ROS_ERROR("Failed to call service get_feedbac with channel number: %d", channel_num);
    }
  }

}  // namespace trog_base
