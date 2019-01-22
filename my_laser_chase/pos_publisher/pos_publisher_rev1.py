import cv2
import numpy as np
from numpy.linalg import inv
import rospy
from geometry_msgs.msg import Pose
import math
import kinematics

# create a grid of the robot workspace, with 1 being reachable and 0 otherwise ---------------------------
grid = np.zeros((50,40,70))
grid_size = 5 #mm

# transformation matrix from robot to grid
r_T_g = np.array([[1,  0, 0, -35*grid_size],
                  [0,  0, 1,            0],
                  [0, -1, 0,          150],
                  [0,  0, 0,            1]])

g_T_r = inv(r_T_g)

# ix is the x-index of the grid
# gx is the x in millimeter in the grid coordinate
# rx is the x in millimeter in the robot coordinate
dummy_angles = [0,0,0]
for ix in range(grid.shape[2]):
    for iy in range(grid.shape[1]):
        for iz in range(grid.shape[0]):
            (gx,gy,gz) = (ix*grid_size, iy*grid_size ,iz*grid_size)
            g_vec = np.array([[gx],[gy],[gz],[1]])
            r_vec = r_T_g.dot(g_vec)
            rx = r_vec[0,0]
            ry = r_vec[1,0]
            rz = r_vec[2,0]
            
            if (not (rx == 0 and ry == 0)) and (kinematics.solve(rx,ry,rz, dummy_angles) == True):
                grid[iz,iy,ix] = 1
#-------------------------------------------------------------------------------------------------------------

# publisher
pub = rospy.Publisher('position', Pose, queue_size=10)
rospy.init_node('pos_publisher', anonymous=True)
rate = rospy.Rate(30) # 10hz

pos = Pose()

# distance in mm between camera and laser dots
depth = 345

# camera matrix obtained from millimeter measurements
k = np.array([[639.93, 0.0,     299.313],
              [0.0,    640.761, 250.992],
              [0.0,    0.0,     1.0    ]])
k_inv = inv(k)

cap = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    ret, tilted_frame = cap.read()
    
    #perspective transform
    rows,cols,ch = tilted_frame.shape
    pts1 = np.float32([[530,172], [623,177], [622,295], [528,298]])
    pts2 = np.float32([[366,165], [501,166], [500,301], [364,300]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    frame = cv2.warpPerspective(tilted_frame,M,(cols,rows))    
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([150,0,252]), np.array([170,15,255]))
    kernel3 = np.ones((3,3), np.uint8)
    morph = cv2.dilate(mask, kernel3, iterations = 3)
    
    #contours
    _, contours,_ = cv2.findContours(morph, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame,contours,-1,(0,255,0),1)    
    
    if len(contours) >  1:
        print('more than 1 contours detected')
    if len(contours) == 1 and cv2.contourArea(contours[0]) < 45:
        print('contour area too small')
    
    #centroid
    if len(contours) == 1 and cv2.contourArea(contours[0]) > 45:
        M = cv2.moments(contours[0])
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        imgPt = np.array([[cx],
                          [cy],
                          [1 ]])
        
        # vector from camera to laser point in real world
        c_vec = k_inv.dot(depth*imgPt)
        
        # add 1 to make it homogeneous coordinate
        c_vec = np.append(c_vec,[[1]],axis=0)
        
        # transformation matrix from robt to camera.
        # x & z of the translation vector is found by shining laser at crosshair and making r_vec zero in x & z
        r_T_c = np.array([[1,  0, 0,   5],
                          [0,  0, 1, -80],
                          [0, -1, 0,  110],
                          [0,  0, 0,    1]])
        
        # vector in the robot coordinate
        r_vec = r_T_c.dot(c_vec)
        print('r_vec in (x,y,z) is ({:.2f}, {:.2f}, {:.2f})'.format(r_vec[0,0], r_vec[1,0], r_vec[2,0]))
        
        g_vec = g_T_r.dot(r_vec)
        gx = g_vec[0,0]
        gy = g_vec[1,0]
        
        ix = int(math.floor(gx/grid_size))
        iy = int(math.floor(gy/grid_size))
        
        if ix > -1 and iy > -1:
            depth_strip = grid[:,iy,ix]
            max_depth_index = 0
            
            for i in range(depth_strip.size-1,-1,-1):
                if depth_strip[i] == 1:
                    max_depth_index = i        
                    break        
            
            ry = max_depth_index * grid_size
            
            if ry != 0:
                pos.position.x = r_vec[0,0]
                pos.position.y = ry
                pos.position.z = r_vec[2,0]
                pub.publish(pos)
            
        rate.sleep()        
        
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()