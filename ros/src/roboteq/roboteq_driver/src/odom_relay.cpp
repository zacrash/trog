
#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"
#include "ros/ros.h"


bool Controller::getFeedback(roboteq_driver::Feedback::Request &req, roboteq_driver::Feedback::Response &res)
{ 
  int channel_num = req.channel;

     if (channel_num >= 1 && channel_num <= channels_.size()) {
       res.measuredPosition = channels_[req.channel-1]->getMeasuredPosition();
     }
     else {
       ROS_WARN("Bad channel number. Dropping message.");
       return false;
     }

    return true;
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_relay");
    ros::NodeHandle n; 
    pub_feedback_ = nh_.advertise<roboteq_msgs::Feedback>("feedback", 1);
    ros::Subscriber sub = n.subscribe("/left/feedback", 1000, chatterCallback);
    ros::Subscriber sub = n.subscribe("/right/feedback", 1000, chatterCallback);
    ros::spin();

  return 0;
}