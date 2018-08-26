
#include <ros/ros.h>
// #include "velodyne_position/rawposition.h"
#include "nmea_msgs/Sentence.h"
#include <velodyne_msgs/VelodynePosition.h>


  // Refer Section 9.3.3 (p. 60) in VLP User Manual for packet format
  typedef struct raw_position_packet
  {
    uint8_t unused0[198];
    uint32_t timestamp;
    uint8_t pps_status;
    uint8_t unused1[3];
    uint8_t nmea_sentence[306];
  } raw_position_packet_t;

  // PPS Status Codes
  const int NO_PPS_DETECTED = 0;
  const int SYNCING_TO_PPS = 1;
  const int PPS_LOCKED = 2;
  const int ERROR = 3;

  class RawPosition
{
  public:
    RawPosition()
    {
      ROS_INFO("Now listening for position packets...");

      //Topic you want to publish
      nmea_ = n_.advertise<nmea_msgs::Sentence>("velodyne_position", 1);

      //Topic you want to subscribe
      raw_position_ = n_.subscribe("velodyne_position_packet", 1, &RawPosition::callback, this);
    }

    void callback(const velodyne_msgs::VelodynePosition& pkt)
    {
          const raw_position_packet_t *raw = (const raw_position_packet_t *) &pkt.data;

          if(raw->pps_status == NO_PPS_DETECTED)
              ROS_INFO("No PPS detected");
          else if (raw->pps_status == SYNCING_TO_PPS)
              ROS_INFO("Synchronizing to PPS...");
          else if (raw->pps_status == PPS_LOCKED)
          {
              ROS_INFO("Publishing nmea data ...");
              ROS_INFO("%s", raw->nmea_sentence);
              // nmea_.publish(raw->nmea_sentence);
          }
          else {
              ROS_ERROR("PPS Error ");
          }
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher nmea_;
    ros::Subscriber raw_position_;

};

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_to_nmea_node");
  // ros::NodeHandle node;
  // ros::NodeHandle priv_nh("~");

  // convert raw position packets to nmea sentences
  RawPosition convert2nmea;

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
