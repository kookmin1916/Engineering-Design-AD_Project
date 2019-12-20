#!/usr/bin/env python

import cv2, time, rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorDetector:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.cap = np.empty(shape=[0])
        rospy.Subscriber(topic, Image, self.conv_image)
   

    def conv_image(self, data):
        self.cap = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def show(self):
        cv2.imshow("Camera", self.cap)

    def find_red(self):
	hsv = cv2.cvtColor(self.cap, cv2.COLOR_BGR2HSV)
	lower_red = np.array([120, 100, 0])
	upper_red = np.array([255, 255, 255])
	binary = cv2.inRange(hsv, lower_red, upper_red)
        result = cv2.bitwise_and(self.cap, self.cap, mask=binary)
	cv2.imshow('red', binary)


	frame_to_thresh = hsv
 
        thresh = cv2.inRange(frame_to_thresh, lower_red, upper_red)
 
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
 
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        
        ret = (None, None)
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            ret = ((x, y), radius)
            M = cv2.moments(c)  
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
            # only proceed if the radius meets a minimum size
            if radius > 5:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(self.cap, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(self.cap, center, 3, (0, 0, 255), -1)
                #cv2.putText(image,"centroid", (center[0]+10,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
                #cv2.putText(image,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)

	cv2.imshow('red', self.cap)
	#print(center)
        return ret 
 
