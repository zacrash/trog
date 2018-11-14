#include <boost/assign/list_of.hpp>
#include <string>
#include "trog_control/trog_hardware.h"
#include <ros/ros.h>

#include "roboteq_msgs/Command.h"

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
  const double WHEEL_DIAMETER = 0.254;
  const double PI  =3.141592653589793238463;

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
                                                              &joints_[i].position, 
                                                              &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

  /**
   *  Get latest velocity commands from ros_control via joint structure, and send to MCU
   */
  void TrogHardware::writeCommandsToHardware()
  {
    double diff_speed_left = joints_[LEFT].velocity_command;
    double diff_speed_right = joints_[RIGHT].velocity_command;

    // Set up messages
    roboteq_msgs::Command cmd_left;
    cmd_left.setpoint = diff_speed_left;

    roboteq_msgs::Command cmd_right;
    cmd_right.setpoint = diff_speed_right;

    //Publish
    left_motor_pub.publish(cmd_left);
    right_motor_pub.publish(cmd_right);
  }

  /**
   *  Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
   */
  void TrogHardware::updateJointsFromHardware()
  {
    ros::ServiceClient client = nh_.serviceClient<roboteq_driver::Feedback>("get_feedback");
    roboteq_driver::Feedback srv;

    for (int channel_num = 1; channel_num <= 2; channel_num++) {
      srv.request.channel = channel_num;
      if (client.call(srv))
      {
        float mp =  srv.response.measuredPosition;
        joints_[channel_num-1].position = channel_num == 1 ? mp : -mp;
      }
      else
        ROS_ERROR("Failed to call service get_feedback with channel number: %d", channel_num);
    }
  }

}  // namespace trog_base
