# miniature-parakeet
For troggie

## User Guide
    # Starts sensors and control nodes
    $ roslaunch trog_bringup bringup.launch 

    # Starts navigation in a known map
    $ roslaunch trog_2dnav known_map.launch


## Tonight's TODOs
    1. Get scanmatcher package.
    2. Make a map.
    3. Get navigation from lab to elevator.
    
## Hardware We Need
* WAITING ON IT: [DC power jack for Jetson TX2](https://www.supercircuits.com/male-power-connector-flying-leads-male-pow?gclid=CjwKCAjwh9_bBRA_EiwApObaOKxp4qUOamEUIJXTlvJDfFYLvvcYt0LzMsHs-ifkTsy37nqZmdrEChoCrnwQAvD_BwE)


## Hardware TODOs
* Connect IMU (RIP) to Jetson
* Establish serial communication with SDC2130
* Hook up encoders

## Software TODOs
* Look into [laser scan matching package](http://wiki.ros.org/laser_scan_matcher)
* Get SLAM working
* Look into a GPS Navigation package
* Finish Arduino Code
