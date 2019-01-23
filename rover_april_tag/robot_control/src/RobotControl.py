#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 19 2016.
"""

import yaml
import numpy as np
import time

import sys

from RosInterface import ROSInterface
import rospy

from ShortestPath import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        Inputs: (all loaded from the parameter YAML file)
        world_map - a P by 4 numpy array specifying the location, orientation,
            and identification of all the markers/AprilTags in the world. The
            format of each row is (x,y,theta,id) with x,y giving 2D position,
            theta giving orientation, and id being an integer specifying the
            unique identifier of the tag.
        occupancy_map - an N by M numpy array of boolean values (represented as
            integers of either 0 or 1). This represents the parts of the map
            that have obstacles. It is mapped to metric coordinates via
            x_spacing and y_spacing
        pos_init - a 3 by 1 array specifying the initial position of the robot,
            formatted as usual as (x,y,theta)
        pos_goal - a 3 by 1 array specifying the final position of the robot,
            also formatted as (x,y,theta)
        max_speed - a parameter specifying the maximum forward speed the robot
            can go (i.e. maximum control signal for v)
        max_omega - a parameter specifying the maximum angular speed the robot
            can go (i.e. maximum control signal for omega)
        x_spacing - a parameter specifying the spacing between adjacent columns
            of occupancy_map
        y_spacing - a parameter specifying the spacing between adjacent rows
            of occupancy_map
        t_cam_to_body - numpy transformation between the camera and the robot
            (not used in simulation)
        """
        
        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        self.kalman_filter = KalmanFilter(world_map)
        self.prev_v = 0
        self.est_pose = np.array([[0], [0], [0]])
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        self.prev_imu_meas = np.array([[0], [0], [0], [0], [0]])

    def process_measurements(self, waypoint):
        """ 
        waypoint is a 1D list [x,y] containing the next waypoint the rover needs to go
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and esimtaiton
        are done. This function is called at 60Hz
        """
        
        #This gives the xy location and the orientation of the tag in the rover frame
        #The orientation is zero when the rover faces directly at the tag
        meas = self.ros_interface.get_measurements()
        
        self.est_pose = self.kalman_filter.step_filter(self.prev_v, self.prev_imu_meas, meas)
        
        state = [self.est_pose[0,0], self.est_pose[1,0], self.est_pose[2,0]]
        goal = [waypoint[0], waypoint[1]]
        controls = self.diff_drive_controller.compute_vel(state,goal)

        self.ros_interface.command_velocity(controls[0], controls[1])
        self.prev_v = controls[0]
        imu_meas = self.ros_interface.get_imu()
        if imu_meas != None:
            imu_meas[3,0] = -imu_meas[3,0]  #clockwise angular vel is positive from IMU
        
        self.prev_imu_meas = imu_meas
        
        return controls[2]
    
def main(args):
    rospy.init_node('robot_control', anonymous = True)
    
    param_path = rospy.get_param("~param_path")
    
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map, occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body)
    
    # Create path
    path = dijkstras(occupancy_map, x_spacing, y_spacing, pos_init, pos_goal)
    print(path)
    
    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    for waypoint in path:
        print(waypoint)
        if rospy.is_shutdown():
            break
        while True and not rospy.is_shutdown():
            done = robotControl.process_measurements(waypoint)
            if done == True:
                break
            r.sleep()

    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    main(sys.argv)

