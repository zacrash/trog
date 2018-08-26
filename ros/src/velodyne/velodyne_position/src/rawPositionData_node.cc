
#include <ros/ros.h>
#include "velodyne_position/rawPositionData.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_to_nmea_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // convert raw position packets to nmea sentences
  velodyne_rawposition::rawPositionData convert2nmea(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
