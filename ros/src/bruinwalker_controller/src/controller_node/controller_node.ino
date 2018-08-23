// Need a better name lols

/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
*/


void setup(){
  Serial1.begin(115200) ; // Roboteq SDC2130 COM (Must be 115200)

  // Give the Roboteq some time to boot-up. 
  delay(1000) ;
}

void loop(){
  delay(1);
}
