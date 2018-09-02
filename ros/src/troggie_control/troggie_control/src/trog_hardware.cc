#include "trog_control/trog_hardware.h"

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
  const double WHEEL_DIAMETER = 0.254;
};

namespace trog_control
{

  /**
  * Initialize Husky hardware
  */
  TrogHardware::TrogHardware(ros::NodeHandle nh, ros::NodeHandle private_nh): nh_(nh), private_nh_(private_nh)
  {
    // TODO: Figure out best parameters
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, WHEEL_DIAMETER);
    private_nh_.param<double>("max_accel", max_accel_, 5.0); 
    private_nh_.param<double>("max_speed", max_speed_, 1.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

     left_motor_pub = nh.advertise<roboteq_msgs::Command>("/left/cmd", 50); //TODO: 50?
     right_motor_pub = nh.advertise<roboteq_msgs::Command>("/right/cmd", 50); //TODO: 50?

    std::string port = "/dev/ttyUSB0";
    int32_t baud = 115200;
    initMotorController(port, baud);
    
    registerControlInterfaces();

    // TODO: Set up encoders
  }

  void TrogHardware::initMotorController(std::string port, int32_t baud)
  {
    // Interface to motor controller.
    controller_ = roboteq::Controller(port.c_str(), baud);

    // Attempt to connect and run.
    while (ros::ok()) {

      ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
      controller.connect();

      if (controller.connected()) {
        ROS_INFO("Connected to roboteq...");
        ros::AsyncSpinner spinner(1);
        spinner.start();
        while (ros::ok()) {
          controller.spinOnce();
        }
        spinner.stop();
      } else {
        ROS_DEBUG("Problem connecting to serial device.");
        ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
        sleep(1);
      }  
    }

  }

  // TODO: Implement for us
  // /**
  // * Get current encoder travel offsets from MCU and bias future encoder readings against them
  // */
  // void HuskyHardware::resetTravelOffset()
  // {
  //   horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy::Channel<clearpath::DataEncoders>::requestData(
  //     polling_timeout_);
  //   if (enc)
  //   {
  //     for (int i = 0; i < 4; i++)
  //     {
  //       joints_[i].position_offset = linearToAngular(enc->getTravel(i % 2));
  //     }
  //   }
  //   else
  //   {
  //     ROS_ERROR("Could not get encoder data to calibrate travel offset");
  //   }
  // }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void TrogHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
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

  // TODO: Read from roboteq
  void TrogHardware::updateJointsFromHardware()
  {

    //   // TODO: Update Joints
    //   // TODO: Which fucking channel. If there are no namespaces. manually use "/channel_1/feedback"
    // //  ROS_DEBUG_STREAM("Received travel information (L:" << msg.measured_position << " R:" << enc->getTravel(RIGHT) << ")");
    //   for (int i = 0; i < 4; i++)
    //   {
    //     double delta = linearToAngular(enc->getTravel(i % 2)) - joints_[i].position - joints_[i].position_offset;

    //     // detect suspiciously large readings, possibly from encoder rollover
    //     if (std::abs(delta) < 1.0)
    //     {
    //       joints_[i].position += delta;
    //     }
    //     else
    //     {
    //       // suspicious! drop this measurement and update the offset for subsequent readings
    //       joints_[i].position_offset += delta;
    //       ROS_DEBUG("Dropping overflow measurement from encoder");
    //     }
    //   }
    // }

    // //TODO: Read velocity from motor controller
    // //horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed = horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::requestData(
    //   polling_timeout_);
    // if (speed)
    // {
    //   ROS_DEBUG_STREAM("Received linear speed information (L:" << speed->getLeftSpeed() << " R:" << speed->getRightSpeed() << ")");
    //   for (int i = 0; i < 4; i++)
    //   {
    //     if (i % 2 == LEFT)
    //     {
    //       joints_[i].velocity = linearToAngular(speed->getLeftSpeed());
    //     }
    //     else
    //     { // assume RIGHT
    //       joints_[i].velocity = linearToAngular(speed->getRightSpeed());
    //     }
    //   }
    // }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void TrogHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    // Set up messages
    roboteq_msgs::Command cmd_left;
    cmd_left.mode = roboteq_msgs::MODE_VELOCITY;
    cmd_left.setpoint = diff_speed_left;

    roboteq_msgs::Command cmd_right;
    cmd_right.mode = roboteq_msgs::MODE_VELOCITY;
    cmd_right.setpoint = diff_speed_right;

    //Publish
    ROS_INFO("Publishing velocity of " + str(cmd_left.setpoint) + " and " + str(cmd_right.setpoint));
    left_motor_pub.publish(cmd_left);
    right_motor_pub.publish(cmd_right);
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void TrogHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * Husky reports travel in metres, need radians for ros_control RobotHW
  */
  double TrogHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, Husky needs m/s,
  */
  double TrogHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace husky_base
