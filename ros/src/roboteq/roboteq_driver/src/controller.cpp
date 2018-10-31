/**
Software License Agreement (BSD)

\file      controller.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
           Mike Irvine <mirvine@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "roboteq_driver/controller.h"
#include "roboteq_driver/channel.h"

#include "roboteq_msgs/Status.h"
#include "serial/serial.h"

// For ROS Service, allows for "on demand" feedback
#include "roboteq_driver/Feedback.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>

// Link to generated source from Microbasic script file.
extern const char* script_lines[];
extern const int script_ver = 28;

namespace roboteq {

const std::string eol("\r");
const size_t max_line_length(128);

Controller::Controller(const char *port, int baud)
  : nh_("~"), port_(port), baud_(baud), connected_(false), receiving_script_messages_(false),
    version_(""), start_script_attempts_(0), serial_(NULL),
    command("!", this), query("?", this), param("^", this)
{
  pub_status_ = nh_.advertise<roboteq_msgs::Status>("status", 1);
}

Controller::~Controller() {
}

void Controller::addChannel(Channel* channel) {
  channels_.push_back(channel);
}

void Controller::connect() {
  if (!serial_) serial_ = new serial::Serial();
  serial::Timeout to(serial::Timeout::simpleTimeout(500));
  serial_->setTimeout(to);
  serial_->setPort(port_);
  serial_->setBaudrate(baud_);

  for (int tries = 0; tries < 5; tries++) {
    try {
      serial_->open();
      query << "FID" << send;
      setSerialEcho(false);
      flush();
    } catch (serial::IOException) {
    }

    if (serial_->isOpen()) {
      connected_ = true;
      return;
    } else {
      connected_ = false;
      ROS_INFO("Bad Connection with serial port Error %s",port_);
    }
  }

  ROS_INFO("Motor controller not responding.");
}

void Controller::read() {
  ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << serial_->available());
  std::string msg = serial_->readline(max_line_length, eol);
  if (!msg.empty()) {
    ROS_DEBUG_STREAM_NAMED("serial", "RX: " << msg);
    if (msg[0] == '+' || msg[0] == '-') {
      // Notify the ROS thread that a message response (ack/nack) has arrived.
      boost::lock_guard<boost::mutex> lock(last_response_mutex_);
      last_response_ = msg;
      last_response_available_.notify_one();
    } else if (msg[0] == '&') {
      receiving_script_messages_ = true;
      // Message generated by the Microbasic script.
      boost::trim(msg);
      if (msg[1] == 's') {
        processStatus(msg);
      } else if (msg[1] == 'f') {
        processFeedback(msg);
      }
    } else {
      // Unknown other message.
//      ROS_WARN_STREAM("Unknown serial message received: " << msg);
    }
  } else {
    ROS_WARN_NAMED("serial", "Serial::readline() returned no data.");
    if (!receiving_script_messages_) {
      if (start_script_attempts_ < 5) {
        start_script_attempts_++;
        ROS_DEBUG("Attempt #%d to start MBS program.", start_script_attempts_);
        startScript();
        flush();
      } else {
        ROS_DEBUG("Attempting to download MBS program.");
        if (downloadScript()) {
          start_script_attempts_ = 0;
        }
        ros::Duration(1.0).sleep();
      }
    } else {
      ROS_DEBUG("Script is believed to be in-place and running, so taking no action.");
    }
  }
}

void Controller::write(std::string msg) {
  tx_buffer_ << msg << eol;
}

void Controller::flush() {
  ROS_DEBUG_STREAM_NAMED("serial", "TX: " << boost::algorithm::replace_all_copy(tx_buffer_.str(), "\r", "\\r"));
  ssize_t bytes_written = serial_->write(tx_buffer_.str());
  if (bytes_written < tx_buffer_.tellp()) {
    ROS_WARN_STREAM("Serial write timeout, " << bytes_written << " bytes written of " << tx_buffer_.tellp() << ".");
  }
  tx_buffer_.str("");
}

void Controller::processStatus(std::string str) {
  roboteq_msgs::Status msg;
  msg.header.stamp = ros::Time::now();

  std::vector<std::string> fields;
  boost::split(fields, str, boost::algorithm::is_any_of(":"));
  try {
    int reported_script_ver = boost::lexical_cast<int>(fields[1]);
    static int wrong_script_version_count = 0;
    if (reported_script_ver == script_ver) {
      wrong_script_version_count = 0;
    } else {
      if (++wrong_script_version_count > 5) {
        ROS_WARN_STREAM("Script version mismatch. Expecting " << script_ver <<
            " but controller consistently reports " << reported_script_ver << ". " <<
            ". Now attempting download.");
        downloadScript();
      }
      return;
    }

    if (fields.size() != 7) {
      ROS_WARN("Wrong number of status fields. Dropping message.");
      return;
    }

    msg.fault = boost::lexical_cast<int>(fields[2]);
    msg.status = boost::lexical_cast<int>(fields[3]);
    msg.ic_temperature = boost::lexical_cast<int>(fields[6]);
  } catch (std::bad_cast& e) {
    ROS_WARN("Failure parsing status data. Dropping message.");
    return;
  }

  pub_status_.publish(msg);
}

void Controller::processFeedback(std::string msg) {
  std::vector<std::string> fields;
  ROS_INFO("Received feedback: %s", msg);
  boost::split(fields, msg, boost::algorithm::is_any_of(":"));
  if (fields.size() != 11) {
    ROS_WARN("Wrong number of feedback fields. Dropping message.");
    return;
  }
  int channel_num;
  try {
    channel_num = boost::lexical_cast<int>(fields[1]);
  } catch (std::bad_cast& e) {
    ROS_WARN("Failure parsing feedback channel number. Dropping message.");
    return;
  }
  if (channel_num >= 1 && channel_num <= channels_.size()) {
    channels_[channel_num - 1]->feedbackCallback(fields);
  } else {
    ROS_WARN("Bad channel number. Dropping message.");
    return;
  }
}

bool Controller::downloadScript() {
  ROS_DEBUG("Commanding driver to stop executing script.");

  // Stop the running script, flag us to start it up again after..
  stopScript();
  flush();
  receiving_script_messages_ = false;

  // Clear serial buffer to avoid any confusion.
  ros::Duration(0.5).sleep();
  serial_->read();

  // Send SLD.
  ROS_DEBUG("Commanding driver to enter download mode.");
  write("%SLD 321654987"); flush();

  // Look for special ack from SLD.
  for (int find_ack = 0; find_ack < 7; find_ack++) {
    std::string msg = serial_->readline(max_line_length, eol);
    ROS_DEBUG_STREAM_NAMED("serial", "HLD-RX: " << msg);
    if (msg == "HLD\r") goto found_ack;
  }
  ROS_DEBUG("Could not enter download mode.");
  return false;
  found_ack:

  // Send hex program, line by line, checking for an ack from each line.
  int line_num = 0;
  while(script_lines[line_num]) {
    std::string line(script_lines[line_num]);
    write(line);
    flush();
    std::string ack = serial_->readline(max_line_length, eol);
    ROS_DEBUG_STREAM_NAMED("serial", "ACK-RX: " << ack);
    if (ack != "+\r") return false;
    line_num++;
  }
  return true;
}


bool Controller::getFeedback(roboteq_driver::Feedback::Request &req, roboteq_driver::Feedback::Response &res)
{ 
  int channel_num = req.channel;


     if (channel_num >= 1 && channel_num <= channels_.size()) {
       res.measuredVelocity = channels_[req.channel-1]->getMeasuredVelocity();
     }
     //else {
    //   ROS_WARN("Bad channel number. Dropping message.");
    //   return false;
    // }
     ROS_INFO("Feedback for Channel %d: %.2f", channel_num, res.measuredVelocity);
   return true;
}

}  // namespace roboteq
