
#include "trog_control/trog_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop for Trog, not realtime safe
*/
void controlLoop(trog_control::TrogHardware &trog,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  trog.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  trog.writeCommandsToHardware();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trog_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  ROS_INFO("Initializing...");
  // Initialize robot hardware and link to controller manager
  trog_control::TrogHardware trog(nh, private_nh);
  controller_manager::ControllerManager cm(&trog, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Husky hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue trog_queue;
  ros::AsyncSpinner trog_spinner(1, &trog_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
    ros::Duration(1 / control_frequency),
    boost::bind(controlLoop, boost::ref(trog), boost::ref(cm), boost::ref(last_time)),
    &trog_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ROS_INFO("Initialization complete...");
  trog_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
