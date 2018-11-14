
#include "ros/ros.h"
#include "roboteq_msgs/Feedback.h"

float left_measured_position;
float right_measured_position;

bool getFeedback(roboteq_driver::Feedback::Request &req, roboteq_driver::Feedback::Response &res)
{ 
  int channel_num = req.channel;

     if (channel_num >= 1 && channel_num <= channels_.size()) {
       res.measuredPosition = channel_num == 1 ? left_measured_position : right_measured_position;
     }
     else {
       ROS_WARN("Bad channel number. Dropping message.");
       return false;
     }

    return true;
}

void saveLeftFeedback(const roboteq_msgs::Feedbackr& fb)
{
   left_measured_position =  fb.measured_position;
}

void saveRightFeedback(const roboteq_msgs::Feedbackr& fb)
{
   right_measured_position =  fb.measured_position; 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_relay");
    ros::NodeHandle n; 

    ros::Subscriber l_feedback = n.subscribe("/left/feedback", 1000, saveLeftFeedback);
    ros::Subscriber r_feedback = n.subscribe("/right/feedback", 1000, saveRightFeedback);

    ros::ServiceServer service = n.advertiseService("get_feedback", &roboteq::Controller::getFeedback, &controller);

    // Handle callbacks
    ros::spin();

  return 0;
}