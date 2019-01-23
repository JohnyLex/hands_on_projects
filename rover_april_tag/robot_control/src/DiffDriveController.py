#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        self.kp=0.2
        self.ka=1.0
        self.kb=0
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        p = (dx**2+dy**2)**0.5
        a = -state[2] + np.arctan2(dy,dx)
        if a < -np.pi:
            a = a + 2*np.pi
        elif a > np.pi:
            a = a - 2*np.pi
        b = -state[2] - a
        
        v = min(self.kp*p, self.MAX_SPEED)
        
        #the motors can stall if v < 0.17
        if v < 0.12:
            v = 0.12
        omega = min(self.ka*a + self.kb*b, self.MAX_OMEGA)
        degCrit = 15.0/180.0*np.pi
        if p < 0.1:
            done = True
        else:
            done = False
        
        return [v,omega,done]
