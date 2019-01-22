import numpy as np
from numpy.linalg import inv

from keras.layers import Conv2D, MaxPooling2D, GlobalAveragePooling2D
from keras.layers import Dropout, Flatten, Dense, Activation, BatchNormalization
from keras.models import Sequential
from keras import regularizers
from glob import glob
import cv2
import pandas as pd
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import rospy

### Inputs ---------------------------------------------------------------------------------------

# For "Trained Model" section
input_shape = (30,30,1)
num_classes = 61
char_names = [item[42:-1] for item in sorted(glob("../char_gen/generated_char_split/train/*/"))]
model_weight_path = '../neural_net_training/saved_models/good1.hdf5'

# For "Obtain letter locations" section
k = np.array([[628.66, 0.0,    312.45],
              [0.0,    629.83, 247.39],
              [0.0,    0.0,     1.0    ]])
k_inv = inv(k)
depth_cam2key = 253 #millimeter

# For "Find red circle location and type" section
# k_inv is used
depth_cam2cir = 202 #millimeter
depth_cir2key = 51
good_inRange_count = 1

### (end) Inputs ---------------------------------------------------------------------------------------

### Trained Model ---------------------------------------------------------------------------------
model = Sequential()

model.add(Conv2D(32, (3, 3), padding='same',
                 input_shape=input_shape))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Dropout(0.2))
model.add(Conv2D(32, (3, 3), padding='same'))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(BatchNormalization())
model.add(Dropout(0.2))

model.add(Conv2D(64, (3, 3), padding='same'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Dropout(0.2))
model.add(Conv2D(64, (3, 3), padding='same'))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(BatchNormalization())
model.add(Dropout(0.2))

model.add(Conv2D(128, (3, 3), padding='same'))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Dropout(0.2))
model.add(Conv2D(128, (3, 3), padding='same'))
model.add(Activation('relu'))

model.add(Flatten())
model.add(BatchNormalization())
model.add(Dropout(0.3))
model.add(Dense(256))
model.add(Activation('relu'))
model.add(BatchNormalization())
model.add(Dropout(0.4))
model.add(Dense(num_classes))
model.add(Activation('softmax'))

model.load_weights(model_weight_path)
### (end) Trained Model ---------------------------------------------------------------------------

### Obtain letter locations -----------------------------------------------------------------------
capital_letters = ['A','B','C','D','E','F','G','H','I','J','K', \
                   'L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z']

cap = cv2.VideoCapture(0)

print("\nWhen the video shows that all the letters are boxed in, select the video window and press 'q'")
image_good = False
while not image_good:
    while True:
        ret, frame = cap.read()

        contourIM = frame.copy()
        gray = cv2.cvtColor(contourIM, cv2.COLOR_BGR2GRAY)
        ret, threshold = cv2.threshold(gray,165,255,cv2.THRESH_BINARY)

        kernel33 = np.ones((3,3),np.uint8)
        kernel22 = np.ones((2,2),np.uint8)
        #mask = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel22)
        mask = cv2.dilate(threshold,kernel33,iterations = 4)

        # findContours() does not modify its input image
        # a contour is the outer pixels surrounding a blob
        _, contours,_ = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_area = pd.Series([cv2.contourArea(contour) for contour in contours])
        contours_area = contours_area[contours_area > 120]
        contours_area = contours_area[contours_area < 390]
        contours = [contours[i] for i in contours_area.index]
        contours_bb = [cv2.boundingRect(cnt) for cnt in contours]
        
        max_score = {}
        loc = {}
        
        # drawContours() modifies its input image
        #cv2.drawContours(contourIM, contours, -1,(0,255,0),1)
        
        for i,(x,y,w,h) in enumerate(contours_bb):  # (x,y) is the top left corner of the box
            # make the bounding box a square
            if h > w:
                diff = h-w
                w = h
                x -= (diff/2 + 1)
            if w > h:
                diff = w-h
                h = w
                y -= diff/2
            
            char_img = gray[y:y+h,x:x+w].copy()
            if char_img.shape[0] == 0 or char_img.shape[1] == 0:
                continue
            char_img = cv2.resize(char_img, (30,30), interpolation=cv2.INTER_CUBIC)
            char_img = char_img.astype(np.float32)
            char_img = char_img/255*2 -1 #shape is now (30,30)
            char_img = np.expand_dims(char_img, axis=0) #shape is now (1,30,30)
            char_img = char_img[..., np.newaxis] #shape is now (1,30,30,1)
            predicted_vec = model.predict(char_img)
            score = np.amax(predicted_vec, axis=1)[0]
            prediction = char_names[np.argmax(predicted_vec)]
        
            if score > max_score.get(prediction, 0) and prediction in capital_letters:
                max_score[prediction] = score
                loc[prediction] = (x,y,w,h)
        
        #if 'sym_num' in max_score: del max_score['sym_num']
        #if 'I' in max_score: del max_score['I']
        for c in loc:
            x,y,w,h = loc[c]
            contourIM = cv2.rectangle(contourIM,(x,y),(x+w,y+h),(0,255,0),1)
            contourIM = cv2.putText(contourIM, '{}'.format(c), (x,y-2), \
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0))
            
        cv2.imshow('contourIM',contourIM)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.imshow('contourIM',contourIM)
    
    while True:
        input1 = raw_input("Does this image capture all the letters correctly?  y or n \n").lower()
        if input1 == 'yes' or input1 == 'y':
            image_good = True
            break
        elif input1 == 'no' or input1 == 'n':
            image_good = False
            print("\nWhen the video captures all the letters correctly, select the video window and press 'q'")
            break
        else:
            print('input not recognized')
        
cv2.destroyAllWindows()
cap.release()

centroid_dict = {}
for c in loc:
    x,y,w,h = loc[c]
    cx, cy = x+w/2, y+h/2
    imgPt_c = np.array([[cx],
                        [cy],
                        [1 ]])
    worldPt_c = k_inv.dot(depth_cam2key * imgPt_c)
    centroid_dict[c] = (worldPt_c[0,0], worldPt_c[1,0])

### (end) Obtain letter locations -----------------------------------------------------------------------

### Find red circle location and type ---------------------------------------------------------------------------
hold = raw_input("Enter any key after setting up Arduino")

rospy.init_node('main1', anonymous=True)

pub = rospy.Publisher('position_diff', Pose, queue_size=1)
pos_diff = Pose()

class Movement:
    def __init__(self):
        self.done_tapping = False
    def callback(self, bool_msg):
        self.done_tapping = bool_msg.data
movement = Movement()
rospy.Subscriber("done_tapping", Bool, movement.callback)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

inRange_count = 0
can_publish = False
while True:
    word = raw_input("Enter a word for the robot arm to type. Letters only. No spaces. Type 'exit' to end program\n").upper()
    if word == "EXIT":
        break
    
    for letter in word:
        while True:
            
            # done_tapping obtained by subcribing to the 'done_tapping' topic from Arduino
            if movement.done_tapping == True:
                movement.done_tapping = False
                break
            
            # the loop is used to clear the buffer so that the latest frame will be read by cap.read(). There are 5 buffer frames, according to a forum post. 
            for i in range(4):
                cap.grab()
            
            ret, frame = cap.read()
            frame_drawn = frame.copy()
            
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_lo = cv2.inRange(frame_hsv, np.array([0,100,130]), np.array([10,255,255]))
            mask_hi = cv2.inRange(frame_hsv, np.array([175,100,130]), np.array([180,255,255]))
            mask = cv2.bitwise_or(mask_lo, mask_hi)
            
            kernel3 = np.ones((3,3), np.uint8)
            kernel5 = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel5)
            canny = cv2.Canny(mask,1000,500)
            circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1, minDist=100,
                                       param1=1000, param2=11, minRadius=5, maxRadius=20)
            
            
            # ensure exactly one red circle is found
            if circles is not None and circles.shape[1]==1:
                # reduce dimensions and convert the (x, y) coordinates and radius of the circles to integers
                circles = np.round(circles[0, :]).astype("int")
                (cir_x, cir_y, cir_r) = circles[0]
                
                # draw and show red circle
                cv2.circle(frame_drawn, (cir_x, cir_y), cir_r, (0, 255, 0), 1)
                cv2.rectangle(frame_drawn, (cir_x - 1, cir_y - 1), (cir_x + 1, cir_y + 1), (0, 128, 255), -1)
                cv2.imshow('img',frame_drawn)
                cv2.imshow('img2',canny)
                
                imgPt_cir = np.array([[cir_x],
                                      [cir_y],
                                      [1]])
                                      
                # convert from pixel to real-world position in millimeter
                worldPt_cir = k_inv.dot(depth_cam2cir * imgPt_cir)
                
                # find the difference between the target 'letter' and the red dot 
                x_diff = centroid_dict[letter][0] - worldPt_cir[0,0]
                x_diff = -x_diff # negated so that this difference can be directly added to the current robot position
                y_diff = centroid_dict[letter][1] - worldPt_cir[1,0]
                print("x_diff: {:6.2f}, y_diff: {:6.2f}".format(x_diff, y_diff))
                
                if abs(x_diff) < 0.9 and abs(y_diff) < 0.9:
                    pos_diff.position.x = 0.0
                    pos_diff.position.y = -11.0
                    pos_diff.position.z = -25.0
                    
                    inRange_count += 1
                    if inRange_count == good_inRange_count:  # only allow publishing when the red dot is in range a certain number of times in a row
                        can_publish = True
                else:
                    pos_diff.position.x = x_diff
                    pos_diff.position.y = y_diff
                    pos_diff.position.z = 0.0
                    can_publish = True
                
                if can_publish:
                    pub.publish(pos_diff)
                    inRange_count = 0
                    can_publish = False
                
                    try:
                        rospy.wait_for_message('done_tapping', Bool, timeout=2)
                    except rospy.ROSException as e:
                        if 'timeout exceeded' in e.message:
                            print('timeout exceeded')
                            continue  # no new waypoint within timeout, looping...
                        else:
                            raise e
                
            elif circles is not None and circles.shape[1]>1:
                print('more than one red circles are found')
                
            elif circles is None:
                print('no circles are found')
            
            if cv2.waitKey(1) == ord('q'):
                break
    
 
cv2.destroyAllWindows()
cap.release()

### Find red circle location and type (end) ---------------------------------------------------------------------------























