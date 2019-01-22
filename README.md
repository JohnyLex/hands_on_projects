# my_laser_chase
In this project, a robot arm stretches out to chase a laser dot. The project involves building a 3-DOF arm, performing inverse kinematics, controlling the arm with Arduino and ROS, processing the webcam video using OpenCV functions such as perspective transform, and calculating the laser position relative to the arm using intrinsic camera matrix and transformation matrices.

# typing_bot
In this project, letters on a keyboard are detected and located, then a robot arm moves to corresponding letters to type out a word from user input.

Deep Learning is applied to detect and locate the keyboard letters. A convolutional neural network is built and trained from scratch. A Python script is written to generate thousands of training images. Data augmentation is used to further diversify the data. OpenCV is used to locate the robot arm and find the regions of interest that will be fed into the neural net. Finally, the Arduino receives the location information through ROS and commands the robot arm to move to the keys using a closed-loop control.
