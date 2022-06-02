#!/usr/bin/env python3
import cv2
import rospy
from std_msgs.msg import Float64MultiArray
import geometry_msgs.msg
x = 0
y = 0

def callback(data):
    
    x = int(data.data[0])
    
    y = int(data.data[1])
    raduis = int(data.data[2])
    path = r'/home/pi/catkin_ws/src/rosttt/scripts/img.png'
    cap = cv2.imread(path)
    dsize = (480,480)
    print(x)
    print(y)
    finalblack = cv2.resize(cap, dsize)
    color = (255,0,0)
    thickness = 2
    finalblack = cv2.circle(finalblack, (x,y), raduis, color, thickness)
    
    print(finalblack.shape)
    cv2.imshow("camera's frame", finalblack)
    cv2.waitKey(1)
    
    
    
def listener():
    rospy.init_node('camerashow', anonymous = True)
    rospy.Subscriber('cameradetect', Float64MultiArray, callback)
    rospy.spin()

if __name__ == "__main__":
    while True:
        listener()
        
