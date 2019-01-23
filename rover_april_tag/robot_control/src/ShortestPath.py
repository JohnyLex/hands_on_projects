#!/usr/bin/python

import numpy as np
from time import sleep


def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    
    (nrows, ncols) = occupancy_map.shape
    parentRow = np.zeros((nrows,ncols)).astype(int)
    parentCol = np.zeros((nrows,ncols)).astype(int)
    
    distFromStart = np.full((nrows,ncols), np.inf)
    
    map = np.zeros((nrows,ncols))
    for i in range(0,nrows):
        for j in range(0,ncols):
            if occupancy_map[i,j] == 0:
                map[i,j] = 1
            else:
                map[i,j] = 2
    
    startcol = np.floor(start[0]/x_spacing).astype(int)
    startrow = np.floor(start[1]/y_spacing).astype(int)
    
    distFromStart[startrow,startcol] = 0
    
    endcol = np.floor(goal[0]/x_spacing).astype(int)
    endrow = np.floor(goal[1]/y_spacing).astype(int)
    
    map[startrow, startcol] = 5
    map[endrow, endcol] = 6
    
    
    while True:
        current = np.argmin(distFromStart)
        (currentrow, currentcol) = np.unravel_index(current, (nrows,ncols))
        
        
        if (currentrow,currentcol) == (endrow,endcol):
            break
        
        map[currentrow,currentcol] = 3
        
        if currentrow-1 >= 0 and (map[currentrow-1,currentcol] == 1 or map[currentrow-1,currentcol] == 6):
            parentRow[currentrow-1,currentcol] = currentrow
            parentCol[currentrow-1,currentcol] = currentcol
            distFromStart[currentrow-1,currentcol] = distFromStart[currentrow,currentcol] + y_spacing
            map[currentrow-1,currentcol] = 4
            
        if currentrow+1 < nrows and (map[currentrow+1,currentcol] == 1 or map[currentrow+1,currentcol] == 6):
            parentRow[currentrow+1,currentcol] = currentrow
            parentCol[currentrow+1,currentcol] = currentcol
            distFromStart[currentrow+1,currentcol] = distFromStart[currentrow,currentcol] + y_spacing
            map[currentrow+1,currentcol] = 4
    
        if currentcol+1 < ncols and (map[currentrow,currentcol+1] == 1 or map[currentrow,currentcol+1] == 6):
            parentRow[currentrow,currentcol+1] = currentrow
            parentCol[currentrow,currentcol+1] = currentcol
            distFromStart[currentrow,currentcol+1] = distFromStart[currentrow,currentcol] + x_spacing
            map[currentrow,currentcol+1] = 4
            
        if currentcol-1 > 0 and (map[currentrow,currentcol-1] == 1 or map[currentrow,currentcol-1] == 6):
            parentRow[currentrow,currentcol-1] = currentrow
            parentCol[currentrow,currentcol-1] = currentcol
            distFromStart[currentrow,currentcol-1] = distFromStart[currentrow,currentcol] + x_spacing
            map[currentrow,currentcol-1] = 4
        
        #map[startrow, startcol] = 5
        #print(distFromStart)
        #print([currentrow, endrow])
        #print([currentcol, endcol])
        #sleep(0.5)
        
        distFromStart[currentrow,currentcol] = np.inf

    if np.isinf(distFromStart[endrow,endcol]):
        route = []
    else:
        #parentx = (parentCol + 0.5)*x_spacing
        #parenty = (parentRow + 0.5)*y_spacing
    
        route = np.array([[currentcol, currentrow]])
        
        while parentCol[route[0,1],route[0,0]] != startcol or parentRow[route[0,1],route[0,0]] != startrow:
            route = np.insert(route,0,[[ parentCol[route[0,1],route[0,0]], parentRow[route[0,1],route[0,0]] ]],axis=0)
    
    print(route)
    route = route.astype(float)
    route[:,0] = (route[:,0]+0.5)*x_spacing
    route[:,1] = (route[:,1]+0.5)*y_spacing
    
    ###add the start and goal position from input
    #if route[0,0] != start[0,0] or route[0,1] != start[1,0]:
    #    route = np.insert(route,0,[[ start[0,0],start[1,0] ]],axis=0)
    #if route[-1,0] != goal[0,0] or route[-1,1] != goal[1,0]:
    #    route = np.append(route,[[ goal[0,0],goal[1,0] ]],axis=0)
    
    route = route.tolist()
    return route
    
def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")
    
    
    
    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")
    
def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)

def test_for_capstone():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([[1, 0, 0, 0, 0, 0, 1],
                          [1, 0, 0, 0, 1, 0, 1],
                          [1, 0, 0, 0, 1, 0, 1],
                          [1, 1, 1, 1, 1, 0, 1],
                          [1, 0, 0, 0, 0, 0, 1],
                          [1, 0, 0, 0, 0, 0, 1],
                          [1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.2
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.1], [0]])
    goal1 = np.array([[0.3], [1.1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)
    print(path1)

def main():
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    test_for_capstone()

