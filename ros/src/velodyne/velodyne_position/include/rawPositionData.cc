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
    uint8_t nmea_gprmc_sentence[306];
  } raw_position_packet_t;


/** \brief Velodyne data conversion class */
  class RawData
  {
  public:

    RawData();
    ~RawData() {}

    /** \brief Set up for data processing.
     *
     *  Perform initializations needed before data processing can
     *  begin:
     *  @param private_nh private node handle for ROS parameters
     *  @returns 0 if successful;
     *           errno value for failure
     */
    int setup(ros::NodeHandle private_nh);
    
    void unpack(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data);
    
} // namespace velodyne_rawdata

