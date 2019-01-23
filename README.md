# my_laser_chase
In this project, a robot arm stretches out to chase a laser dot. The project involves building a 3-DOF arm, performing inverse kinematics, controlling the arm with Arduino and ROS, processing the webcam video using OpenCV functions such as perspective transform, and calculating the laser position relative to the arm using intrinsic camera matrix and transformation matrices.

See [YouTube Demo](https://youtu.be/46Q9ypHZdVk)

# typing_bot
In this project, letters on a keyboard are detected and located, then a robot arm moves to corresponding letters to type out a word from user input.

Deep Learning is applied to detect and locate the keyboard letters. A convolutional neural network is built and trained from scratch. A Python script is written to generate thousands of training images. Data augmentation is used to further diversify the data. OpenCV is used to locate the robot arm and find the regions of interest that will be fed into the neural net. Finally, the Arduino receives the location information through ROS and commands the robot arm to move to the keys using a closed-loop control.

See [YouTube Demo](https://youtu.be/I7_Z-FtJ_zA)

# rover_april_tag
Given a map and April Tags, the rover can find a path, localize itself, and navigate to the goal autonomously. Extended Kalman Filter is used to estimate the rover’s location by combining information from April Tags, angular velocity from IMU, and linear velocity from controls. The rover uses a Raspberry Pi for processing and a differential drive for movement. The map is discretized and Dijkstra’s algorithm is applied to find the shortest path between start and goal.

See [YouTube Demo](https://youtu.be/1QYWjZRNBkY)
