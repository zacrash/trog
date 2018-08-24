/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "velodyne_driver/driver.h"

namespace velodyne_driver
{

class DriverNodelet: public nodelet::Nodelet
{
public:

  DriverNodelet():
    running_(false)
  {}

  ~DriverNodelet()
  {
    if (running_)
      {
        NODELET_INFO("shutting down driver thread");
        running_ = false;
        deviceDataThread_->join();
        deviceGpsThread_->join();
        NODELET_INFO("driver thread stopped");
      }
  }

private:

  virtual void onInit(void);
  virtual void deviceDataPoll(void);
  virtual void deviceGpsPoll(void);


  volatile bool running_;               ///< device thread is running
  boost::shared_ptr<boost::thread> deviceDataThread_;
  boost::shared_ptr<boost::thread> deviceGpsThread_;

  boost::shared_ptr<VelodyneDriver> dvr_; ///< driver implementation class
};

void DriverNodelet::onInit()
{
  // start the driver
  dvr_.reset(new VelodyneDriver(getNodeHandle(), getPrivateNodeHandle()));

  // spawn device data poll thread
  running_ = true;
  deviceDataThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&DriverNodelet::deviceDataPoll, this)));

  // spawn device gps poll thread
  deviceGpsThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&DriverNodelet::deviceGpsPoll, this)));
}

/** @brief Device poll thread data loop. */
void DriverNodelet::deviceDataPoll()
{
  while(ros::ok())
    {
      // poll device until end of file
      running_ = dvr_->poll();
      if (!running_)
        break;
    }
  running_ = false;
}

/** @brief Device poll thread gps loop. */
void DriverNodelet::deviceGpsPoll()
{
  while(ros::ok())
    {
      // poll device until end of file
      running_ = dvr_->gpsPoll();
      if (!running_)
        break;
    }
  running_ = false;
}

} // namespace velodyne_driver

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_driver::DriverNodelet, nodelet::Nodelet)
