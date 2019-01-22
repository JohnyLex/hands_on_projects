import cv2
import numpy as np
from numpy.linalg import inv
import rospy
from geometry_msgs.msg import Pose
import math
import kinematics

#### Input Parameters ------------------------------------------------------------------------------------------------------------------

# Robot coordinate system (look at the Readme file for better description with picture)
#   It is located at the beginning of the shoulder link
#   When looking at the robot's back, x-axis points to right, y-axis forward, z-axis upward

# Grid coordinate system (look at the Readme file for better description with picture)
#   It is at the top left corner of the robot when looking at the robot's back
#   When looking at the robot's back, z-axis points to the board, y-axis downward, x-axis right

# number of grid cells in the x,y,z directions in grid coordinate system
(x_size, y_size, z_size) = (70, 40, 50)

# length per grid cell in millimeter
lenPerCell = 5

# position of grid origin in robot CS (coordinate system) in millimeter.
(x_grid_origin, y_grid_origin, z_grid_origin) = (-35*lenPerCell, 0, y_size*lenPerCell - 50)

# transformation matrix from robot CS to grid CS
r_T_g = np.array([[1,  0, 0, x_grid_origin],
                  [0,  0, 1, y_grid_origin],
                  [0, -1, 0, z_grid_origin],
                  [0,  0, 0,             1]])

# transformation matrix from grid CS to robot CS
g_T_r = inv(r_T_g)

# distance in mm between the virtual head-on camera and the laser board
depth = 345

# intrinsic camera matrix obtained using millimeter measurements
k = np.array([[639.93, 0.0,     299.313],
              [0.0,    640.761, 250.992],
              [0.0,    0.0,     1.0    ]])

# inverse of intrinsic camera matrix
k_inv = inv(k)


# corners of the pink sticker when camera is looking at the board at an angle
tiltedCorners = np.float32([[519,123], [613,128], [610,246], [516,250]])
# corners of the pink sticker when camera is looking at the board head-on
headonCorners = np.float32([[386,117], [527,116], [524,255], [385,255]])

# transformation matrix from robt to camera:
#    It is difficult to measure the distance between the robot origin and the virutal head-on camera origin,
#    so I shine the laser at the crosshair of the board when running this python file.
#    The crosshair intersects with the y-axis of the robot coordinate system.
#    r_vec, which is the laser location in the robot CS, is printed out in the console.
#    x & z of the translation vector is found by making x & z of r_vec zero.
#    y is found by making y of r_vec the distance between robot and board (about 263mm).
r_T_c = np.array([[1,  0, 0,  -7],
                  [0,  0, 1, -80],
                  [0, -1, 0,  83],
                  [0,  0, 0,   1]])

#### (END) Input Parameters ------------------------------------------------------------------------------------------------------------


#### Functions -------------------------------------------------------------------------------------------------------------------------

def createWorkspaceGrid(x_size, y_size, z_size, r_T_g, lenPerCell):
    """
    Returns a grid (3D numpy array) with 1 being reachable and 0 otherwise
    The grid's origin is described by r_T_g,
    The grid has the size specified by the arguements,

    The origin position is at the top left corner of the robot when looking at the robot's back
    when looking at the robot's back, z-axis points to the board, y-axis downward, x-axis right

    Arguments:
    x_size, y_size, z_size are integers representing the number of grid cells in z, y, x directions.
    r_T_g is a 4x4 numpy array representing the transformation matrix from robot to grid
    
    """
    
    # Note that the convention is grid[z,y,x]
    grid = np.zeros( (z_size, y_size, x_size) )
    
    # this is needed to use kinematics.solve() below
    dummy_angles = [0,0,0]
    
    """
    The for-loop loops through each grid cell,
    converts it to millimeters,
    transforms it from grid coordinate system to robot coordinate system,
    and calls kinematics.solve() to determine if the cell is reachable
    
    # ix,iy,iz are the x,y,z-indices of the grid. Note that the convention is grid[z,y,x] for numpy arrays
    # gx,gy,gz are the x,y,z in millimeters in the grid coordinate system. g_vec is a column vector containing them
    # rx,ry,rz are the x,y,z in millimeters in the robot coordinate system . r_vec is a column vector containing them
    """
    for ix in range(grid.shape[2]):
        for iy in range(grid.shape[1]):
            for iz in range(grid.shape[0]):
                
                # convert them to millimeters and make a column vector
                (gx,gy,gz) = (ix*lenPerCell, iy*lenPerCell ,iz*lenPerCell)
                g_vec = np.array([[gx],[gy],[gz],[1]])
                
                # transform to robot coordinate system
                r_vec = r_T_g.dot(g_vec)
                rx = r_vec[0,0]
                ry = r_vec[1,0]
                rz = r_vec[2,0]
                
                # solve: return 1 if reachable, 0 otherwise
                if (not (rx == 0 and ry == 0)) and (kinematics.solve(rx,ry,rz, dummy_angles) == True):
                    grid[iz,iy,ix] = 1
    
    return grid

def getLaserContour(tilted_frame, tiltedCorners, headonCorners):
    """
    The OpenCV contour is needed to calculate the center of an area
    
    Arguments:
    tilted_frame is the 3D numpy array captured from the tilted camera
    tiltedCorners is a 4x1 numpy array representing the corners of the pink sticker when camera is looking at the board at an angle
    headonCorners is a 4x1 numpy array represents the corners of the pink sticker when camera is looking at the board headon
    
    Returns:
    contourGood is a Boolean. It means "is the obtained laser contour good?"
    contour
      -It is an element of the list "contours". contour itself is a list of coordinates that surrounds the laser dot.
      -It is in the coordinate system of the image plane of the virtual head-on camera (the red CS in the Readme file)
    frame is a 3D numpy array. contours have been drawn on the frame
    """
    # perspective transform on the tilted_frame. The new perspective faces the laser board head-on
    rows,cols,ch = tilted_frame.shape
    M = cv2.getPerspectiveTransform(tiltedCorners,headonCorners)
    frame = cv2.warpPerspective(tilted_frame,M,(cols,rows))    
    
    # filter the frame to obtain the laser dot (with noise)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    filtered = cv2.inRange(hsv, np.array([150,0,252]), np.array([170,15,255]))
    
    # dilate the frame so that the pixels at the laser dot joins together
    kernel3 = np.ones((3,3), np.uint8)
    morph = cv2.dilate(filtered, kernel3, iterations = 3)
    
    # return "contours", which is a list. Each element is a contour. Each contour itself is a list of coordinates.
    _, contours,_ = cv2.findContours(morph, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    # directly modifies "frame" by drawing all contours on it
    cv2.drawContours(frame,contours,-1,(0,255,0),1) 
    
    # contour is considered good if there is only one contour in the contours list and its area is bigger than 45
    if len(contours) == 1 and cv2.contourArea(contours[0]) > 45:
        contourGood = True
        contour = contours[0]
        return contourGood, contour, frame
    else:
        contourGood = False
        contour = None
        
        # for debug only
        if len(contours) >  1:
            print('More than 1 contours detected.')
        elif len(contours) == 1 and cv2.contourArea(contours[0]) < 45:
            print('Contour area too small')
            
        
    return contourGood, contour, frame
    
    
        
def getLaserPosition(contour, r_T_c):
    """
    Argument:
    contour
      -It is an element of the list "contours". contour itself is a list of coordinates that surrounds the laser dot.
      -It is in the coordinate system of the image plane of the virtual head-on camera (the red CS in the Readme file)
    
    Return:
    r_vec is the 4x1 homogenous vector representing the laser location in the robot coordinate system (the green CS in the Readme file)
    """
    
    # laser contour centroid (cc) in pixels in the coordinate system of the image plane of the virtual head-on camaer (the red CS in the Readme file)
    M = cv2.moments(contour)
    cc_x = int(M['m10']/M['m00'])
    cc_y = int(M['m01']/M['m00'])
    
    # create homogeneous image point using centroid center and 1.
    imgPt = np.array([[cc_x],
                      [cc_y],
                      [1 ]])
    
    # the laser location in the CS of virtual head-on camera. (the purple CS in the Readme file)
    c_vec = k_inv.dot(depth*imgPt) # k_inv is inverse of the intrinsic camaera matrix
    
    # add 1 to make it homogeneous coordinate
    c_vec = np.append(c_vec,[[1]],axis=0)
    
    # laser location in the robot coordinate system (the green CS in the Readme file)
    r_vec = r_T_c.dot(c_vec)
    print('r_vec in (x,y,z) is ({:.2f}, {:.2f}, {:.2f})'.format(r_vec[0,0], r_vec[1,0], r_vec[2,0]))
    
    return r_vec

def getRobotPosition(r_vec, x_size, y_size):
    """
    Argument:
    r_vec is the 4x1 homogenous vector representing the laser location in the robot coordinate system (the green CS in the Readme file)
    x_size and y_size are the sizes of the grid in the grid coordinate system (the blue CS in the Readme file)
    Return:
    positionGood is a Boolean. It means "is the robot position good?"
    pos is the robot position
    """
    
    pos = Pose()
    # g_vec is the vector pointing from grid origin to the laser dot in the grid coordinate system (the blue CS in the Readme file)
    g_vec = g_T_r.dot(r_vec)
    gx = g_vec[0,0]
    gy = g_vec[1,0]
    
    # get indices in the grid coordinate system (the blue CS in the Readme file)
    ix = int(math.floor(gx/lenPerCell))
    iy = int(math.floor(gy/lenPerCell))
    
    positionGood = False
    
    # prevent negative indices and out-of-bound indices
    if ix > -1 and ix < x_size and iy > -1 and iy < y_size:
        depth_strip = grid[:,iy,ix]
        
        max_depth_index = 0
        
        # find, if any, the index that corresponds to the max depth the robot reach while maintaining the planar position.
        for i in range(depth_strip.size-1,-1,-1):
            if depth_strip[i] == 1:
                max_depth_index = i        
                break
        
        # when max_depth_index is not zero, a reachable position is found in the depth strip
        if max_depth_index != 0:
            pos.position.x = r_vec[0,0]
            
            ry = max_depth_index * lenPerCell
            pos.position.y = ry
            
            pos.position.z = r_vec[2,0]
            
            positionGood = True
        else:
            print("no reachable position found")
    else:
        print("laser position out of the grid")
    
    return positionGood, pos

#### (END) Functions ---------------------------------------------------------------------------------------------------------------------

#### Main --------------------------------------------------------------------------------------------------------------------------------

grid = createWorkspaceGrid(x_size, y_size, z_size, r_T_g, lenPerCell)

# publisher
rospy.init_node('pos_publisher', anonymous=True)
pub = rospy.Publisher('position', Pose, queue_size=10)
rate = rospy.Rate(30) # hz

cap = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    ret, tilted_frame = cap.read()        
    contourGood, contour, frame = getLaserContour(tilted_frame, tiltedCorners, headonCorners)
    
    if contourGood == True:
        r_vec = getLaserPosition(contour, r_T_c)
    
        positionGood, pos = getRobotPosition(r_vec,x_size,y_size)
        
        if positionGood == True:
            pub.publish(pos)
                
    rate.sleep()        
    
    cv2.imshow('tilted_frame',tilted_frame)    
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()

#### (END) Main --------------------------------------------------------------------------------------------------------------------------------
