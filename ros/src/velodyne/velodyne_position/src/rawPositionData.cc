  #include <velodyne_position/rawPositionData.h>

namespace velodyne_rawposition {

    /** @brief Constructor. */
    RawPosition::RawPosition(ros::NodeHandle node, ros::NodeHandle private_nh)
    {
        // advertise output point cloud (before subscribing to input data)
        // Needs a nmea_msgs/Sentence
        nmea_ =
            node.advertise<>("nmea_msgs/Sentence", 10); // TODO: Can be 1?

        position_packet_
            = node.subscribe("velodyne_position_packet", 10, unpack);
    }

    void RawPosition::unpack(const velodyne_msgs::VelodynePosition &pkt)
    {
        const raw_packet_t *raw = (const raw_packet_t *) &pkt.data;

        if(raw->pps_status == NO_PPS_DETECTED) 
            ROS_INFO("No PPS detected");
        else if (raw->pps_status == SYNCING_TO_PPS)
            ROS_INFO("Synchronizing to PPS...");
        else if (raw->pps_status == PPS_LOCKED)
        {
            ROS_INFO("Publishing nmea data ...");
            nmea_.publish(raw->nmea_sentence);
        }
        else {
            ROS_ERROR("PPS Error ");
        }
    }
}    

