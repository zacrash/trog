#include <stdint.h>

#include <ros/ros.h>

  namespace velodyne_rawposition
{
  // Refer Section 9.3.3 (p. 60) in VLP User Manual for packet format
  typedef struct raw_position_packet
  {
    uint8_t unused[198];
    uint32_t timestamp;
    uint8_t pps_status; 
    uint24_t unused;
    uint8_t nmea_sentence[306];
  } raw_position_packet_t; 

  // PPS Status Codes
  const int NO_PPS_DETECTED = 0;
  const int SYNCING_TO_PPS = 1;
  const int PPS_LOCKED = 2;
  const int ERROR = 3;

  // Raw Position Data to NMEA Class
  class RawPosition 
  {
    public:
      RawPosition();
      ~RawPosition() {}
      void unpack(const velodyne_msgs::VelodynePosition &pkt);
    
    private:
      ros::Publisher nmea_;
      ros::Subscriber position_packet;
  }

} // namespace velodyne_rawdata

