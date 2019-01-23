# Overview
This is the final project for the Coursera Robotics Specialization taught by University of Pennsylvania.

Given a map and April Tags, the rover can find a path, localize itself, and navigate to the goal autonomously. Extended Kalman Filter is used to estimate the rover’s location by combining information from April Tags, angular velocity from IMU, and linear velocity from controls. The rover uses a Raspberry Pi for processing and a differential drive for movement. The map is discretized and Dijkstra’s algorithm is applied to find the shortest path between start and goal.

See [YouTube Demo](https://youtu.be/1QYWjZRNBkY)

# Hardware List
[Mini Robot Rover Chassis Kit](https://www.adafruit.com/product/2939)\
[USB Battery Pack](https://www.adafruit.com/product/1959)\
[Raspberry Pi 3 - Model B](https://www.adafruit.com/product/3055)\
[Adafruit DC & Stepper Motor HAT for Raspberry Pi](https://www.adafruit.com/product/2348)\
[Brass M2.5 Standoffs for Pi HATs](https://www.adafruit.com/product/2336)\
[Adjustable Pi Camera Mount](https://www.adafruit.com/product/1434)\
[Male-Male Jumper Wires](https://www.adafruit.com/product/1957)\
[AA Battery Holder](https://www.adafruit.com/product/830)\
[MicroSD Memory Card - 16GB](https://www.adafruit.com/product/2693)\
[Raspberry Pi Camera](https://www.adafruit.com/product/3099)\
[Adafruit Precision NXP 9-DOF Breakout Board](https://www.adafruit.com/product/3463)

# Flashing Raspberry Pi SD Card
Download the file [here](https://d18ky98rnyall9.cloudfront.net/_aaf798f8a9b9dc160c0540dfb59115ae_coursera_robotics_capstone_pi.zip?Expires=1548374400&Signature=HGl0vM-gtdbpO-0owbbSREmklGSr2BZXuRpvVdBUIsre3ekkJYSdY-9EDtxd3Lay2LC7zLKFk2HX0FQaZJ3JNzp-PVkMydqBGmjmpR-hdUiM7rfzlcrLW9f25ZHSgBBGvPT0NMkACRzxD~i81dYJABjMb15CW9xvnMcUhvu-Fpg_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A) and flash it to the SD card.\
Note that this is a large file (around 2.2 GB), so please make sure your network can support this.

# Setup
- The files and folders in this repository should go under the _catkin_ws/src_ folder.
- `cd catkin_ws`
- `catkin_make`

# Launching the System
- `roslaunch robot_launch robot.launch`
- `roslaunch robot_control robot_control.launch`
