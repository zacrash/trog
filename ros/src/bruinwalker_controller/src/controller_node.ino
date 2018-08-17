
/*==========================================================================
 Arduino ROS node for Bruinwalker project                                   
 The Arduino controls a SDC2130 Roboteq Motor Controller with Encoder Input 
 SMERC (2018)                                                               

PIN LAYOUT
   SDC2130 DB-15      -------------     Arduino 
   -----------------                    -------
            6*     -------------------   Arduino Rx (D0)
            7*     -------------------   Arduino Tx (D1)

   SDC2130 DB-15      -------------     Encoder Pull Up 
   -----------------                    ---------------
            4      -------------------   Right Motor Encoder A Channel
            8      -------------------   Right Motor Encoder B Channel
            14     -------------------   Right Motor Encoder 5V
            13     -------------------   Right Motor Encoder Ground
            10     -------------------   Left Motor Encoder A Channel
            15     -------------------   Left Motor Encoder B Channel
            14     -------------------   Left Motor Encoder 5V
            13     -------------------   Right Motor Encoder Ground
            
   * NOTE: Pins 6 and 7 are for TTL level serial. Pins 2 and 3 are also
           labeled as Tx and Rx in the motor controllers datasheet but 
           these are RS-232 level.  
=============================================================================*/           

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg); 

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// TODO: Handle Twist commands
void driveCallback ( const geometry_msgs::Twist&  twistMsg )
{
  
  // int steeringAngle = fmap(twistMsg.angular.z, 0.0, 1.0, minSteering, maxSteering) ;
  // // The following could be useful for debugging
  // // str_msg.data= steeringAngle ;
  // // chatter.publish(&str_msg);
  // // Check to make sure steeringAngle is within car range
  // if (steeringAngle < minSteering) { 
  //   steeringAngle = minSteering;
  // }
  // if (steeringAngle > maxSteering) {
  //   steeringAngle = maxSteering ;
  // }
  // steeringServo.write(steeringAngle) ;
  
  // // ESC forward is between 0.5 and 1.0
  // int escCommand ;
  // if (twistMsg.linear.x >= 0.5) {
  //   escCommand = (int)fmap(twistMsg.linear.x, 0.5, 1.0, 90.0, maxThrottle) ;
  // } else {
  //   escCommand = (int)fmap(twistMsg.linear.x, 0.0, 1.0, 0.0, 180.0) ;
  // }
  // // Check to make sure throttle command is within bounds
  // if (escCommand < minThrottle) { 
  //   escCommand = minThrottle;
  // }
  // if (escCommand > maxThrottle) {
  //   escCommand = maxThrottle ;
  // }
  // // The following could be useful for debugging
  // // str_msg.data= escCommand ;
  // // chatter.publish(&str_msg);
  
  // electronicSpeedController.write(escCommand) ;
  // digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

// TODO: What is our twist command called?
ros::Subscriber<geometry_msgs::Twist> twistSubscriber("TWIST CMD", &driveCallback) ;

void setup(){
  Serial.begin(115200); // Roboteq SDC2130 COM (Must be 115200)

  // ROS Initialization
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(twistSubscriber);

  // Attach the servos to actual pins
  steeringServo.attach(9); // Steering servo is attached to pin 9
  electronicSpeedController.attach(10); // ESC is on pin 10
  // Initialize Steering and ESC setting
  // Steering centered is 90, throttle at neutral is 90
  steeringServo.write(90) ;
  electronicSpeedController.write(90) ;

  // Give the Roboteq some time to boot-up. 
  delay(1000) ;
}

void loop(){
  nh.spinOnce();
  delay(1);
}