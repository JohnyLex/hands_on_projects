#!/usr/bin/python
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
import time
import math

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None # Used to keep track of time between measurements 
        self.Q_t = np.diag([0.03, 0.3])
        self.R_t = np.diag([0.05, 0.19, np.deg2rad(30.0)])
        
        self.mean = np.array([[0], [0], [0]])
        self.sigma = np.diag([10**6, 10**6, 10**6]) 
        self.prev_t = time.time()

    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predction of the state
        predicted_covariance - a 3 by 3 numpy array of the predction of the
            covariance
        """
        
        theta = self.mean[2, 0]
        w = imu_meas[3,0]
        
        t_diff = time.time()-self.prev_t
        predicted_mean = self.mean + t_diff*np.array([[v*np.cos(theta)], [v*np.sin(theta)], [w]])
        self.prev_t = time.time()
        
                
        if predicted_mean[2,0] < 0:
            predicted_mean[2,0] = predicted_mean[2,0] + 2*np.pi
        elif predicted_mean[2,0] >= 2*np.pi:
            predicted_mean[2,0] = predicted_mean[2,0] - 2*np.pi
        
        dfdx = np.eye(3) + np.array([[0, 0, -v*np.sin(theta)],
                                     [0, 0,  v*np.cos(theta)],
                                     [0, 0,                0]])
        
        #note that the powerpoint says dfdn, but it should be dfdu with u being the control vector
        dfdu = t_diff*np.array([[np.cos(theta), 0],
                              [np.sin(theta), 0],
                              [0,             1]])
                              
        predicted_sigma= dfdx.dot(self.sigma).dot(dfdx.transpose()) + dfdu.dot(self.Q_t).dot(dfdu.transpose())
        
        self.mean = predicted_mean
        self.sigma = predicted_sigma
        
        return (self.mean, self.sigma)

    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        
        dhdx = np.eye(3)
        dhdx_trans = dhdx
        K_part = inv( dhdx.dot(self.sigma).dot(dhdx_trans) + self.R_t )
        K = self.sigma.dot(dhdx_trans).dot(K_part)
        
        row_idx = np.argwhere(self.markers[:,3] == z_t[0,3])[0,0]
        marker = self.markers[row_idx,:]
        x_w = marker[0]
        y_w = marker[1]
        th_w = marker[2]
        w_H_t = np.array([[np.cos(th_w), -np.sin(th_w), x_w],
                          [np.sin(th_w),  np.cos(th_w), y_w],
                          [           0,             0,   1]])
                         
        
        x_r = z_t[0,0]
        y_r = z_t[0,1]
        th_r = z_t[0,2]
        t_H_r = inv(np.array([[np.cos(th_r), -np.sin(th_r), x_r],
                              [np.sin(th_r),  np.cos(th_r), y_r],
                              [           0,             0,   1]]))
                              
        w_H_r = np.dot(w_H_t, t_H_r)
        
        #robot location in the world frame
        z_w = np.array([[w_H_r[0,2]], [w_H_r[1,2]], [np.arctan2(w_H_r[1,0],w_H_r[0,0])]])
        if z_w[2,0] < 0:
            z_w[2,0] = z_w[2,0] + 2*np.pi
        elif z_w[2,0] >= 2*np.pi:
            z_w[2,0] = z_w[2,0] - 2*np.pi
        print("z_t")
        print(z_t)
        print("z_w")
        print(z_w)

        predicted_mean = self.mean
        
        diff = z_w-predicted_mean
        if diff[2,0] < -np.pi:
            diff[2,0] = diff[2,0] + 2*np.pi
        elif diff[2,0] > np.pi:
            diff[2,0] = diff[2,0] - 2*np.pi
        
        fused_mean = self.mean + np.dot(K, diff)
        fused_sigma = self.sigma - K.dot(dhdx).dot(self.sigma)
        
        self.mean = fused_mean
        self.sigma = fused_sigma
        
        return (self.mean, self.sigma)
        
    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        t, previous time
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        
        #the IMU sometimes reads -34 and +34 rad/s. And >10 isn't realistic
        if imu_meas != None and abs(imu_meas[3,0]) < 10:  
            self.prediction(v, imu_meas)
        else:
            self.sigma = 1.5*self.sigma
        if z_t != None:
            z_t = np.asarray(z_t)
            z_t = z_t[None,0,:]
            self.update(z_t)
            
        return self.mean
