#!/usr/bin/env python3
import rospy
from smbus import SMBus
from std_msgs.msg import Float64MultiArray
import geometry_msgs.msg
import time
import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(26,GPIO.OUT)
clientAddr = 0x0b
bus = SMBus(1)

tolerance = 5
xSet = 80
ySet = 140
errorx = 0
errory = 0
xtoMove = xSet
ytoMove = ySet
x = 0
y = 0
X_lock = 0
Y_lock = 0


servo1 = Servo(17, pin_factory=factory)
inval1 = (ySet/90) - 1
servo1.value = inval1
servo2 = Servo(12, pin_factory=factory)
inval2 = (xSet/90) - 1
servo2.value = inval2
angleval1 = 0
angleval2 = 0

def i2cWrite(msg):
  for c in msg:
    bus.write_byte(clientAddr, ord(c))
  return -1

def callback(data):
    global ytoMove
    global xtoMove
    global xSet
    global ySet
    global errory
    global errorx
    global inval1
    global inval2
    
    x = int(data.data[0])
    y = int(data.data[1])
    if y < (240 - tolerance):
        errory = (240 - y)/60
        ytoMove = ytoMove - int(round(errory))
        inval1 = (ytoMove/90) - 1
        if inval1 > 1:
            inval1 = 1
        elif inval1 < -1:
            inval1 = -1
        servo1.value = inval1
        Y_lock = 0
    elif y > (240 + tolerance):
        errory = (y - 240)/60
        ytoMove = ytoMove + int(round(errory))
        inval1 = (ytoMove/90) - 1
        if inval1 > 1:
            inval1 = 1
        elif inval1 < -1:
            inval1 = -1
        servo1.value = inval1
        Y_lock = 0
    else:
        Y_lock = 1
        
    if x < (240 - tolerance):
        errorx = (240 - x)/48
        xtoMove = xtoMove + int(round(errorx))
        inval2 = (xtoMove/90) - 1
        if inval2 > 1:
            inval2 = 1
        elif inval2 < -1:
            inval2 = -1
        servo2.value = inval2
        X_lock = 0
    elif x > (240 + tolerance):
        errorx = (x - 240)/48
        xtoMove = xtoMove - int(round(errorx))
        inval2 = (xtoMove/90) - 1
        if inval2 > 1:
            inval2 = 1
        elif inval2 < -1:
            inval2 = -1
        servo2.value = inval2
        X_lock = 0
    else:
        X_lock = 1
    
    if int(round(errorx)) == 0 and int(round(errory)) == 0:
        print('locked')
        #i2cWrite('s')
        GPIO.output(26,GPIO.HIGH)
    else:
        print(int(round(errorx)))
        #i2cWrite('a')
        GPIO.output(26,GPIO.LOW)
    
    
    
    
    
def listener():
    rospy.init_node('servocontrol', anonymous = True)
    rospy.Subscriber('cameradetect', Float64MultiArray, callback)
    rospy.spin()
    
if __name__ == '__main__':
    while True:
        xtoMove = xSet
        ytoMove = ySet
        listener()
        #callback()
