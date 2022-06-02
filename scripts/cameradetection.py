#!/usr/bin/env python3
import cv2
import rospy
from std_msgs.msg import Float64MultiArray
import geometry_msgs.msg
import time
import argparse
import imutils
from collections import deque
import numpy


def cameradetect():
    pub = rospy.Publisher('cameradetect', Float64MultiArray, queue_size=10)
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        my_msg = Float64MultiArray()
        cap = cv2.VideoCapture(0)
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, (24,100,100), (44,255,255))
                mask = cv2.erode(mask, None, iterations = 2)
                mask = cv2.dilate(mask, None, iterations = 2)
                mask = cv2.resize(mask, (480,480), interpolation = cv2.INTER_AREA)
                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                if len(cnts) > 0:
                    c = max(cnts, key = cv2.contourArea)
                    ((X,Y), radius) = cv2.minEnclosingCircle(c)
                    d=[X, Y, radius]
                    my_msg.data = d
                    pub.publish(my_msg)
                    
                    
                    print(X)
                    print(Y)
                    
        
            
            else:
                cap.release()
                cv2.destroyAllWindows()



if __name__ == "__main__":
    
    try:
       cameradetect()
    except rospy.ROSInterruptException:
        camera.release()
        cv2.destroyAllWindows()
        pass
        
