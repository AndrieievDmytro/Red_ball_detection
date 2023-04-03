"""PID controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
from controller import Robot
import numpy as np
import argparse
import cv2 as cv
import math
import struct
from controller import *

TIME_STEP = 32
robot = Robot()
wheels = []
wheelsNames = ['lMotor', 'rMotor']

maxVelocity = 10

velocity = 7
kp = 0.07

tau = 0
negative = False

lastError = None
error = None

lower_red = np.array([160,50,50])
upper_red = np.array([180,255,255])
kernel = np.ones((9, 9), np.uint8)

def get_error(frame):

    err = None
    # blured = cv.GaussianBlur(frame, (9,9), 0)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_red, upper_red)
    mask_without_noise = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel) 
    mask_close = cv.morphologyEx(mask_without_noise, cv.MORPH_CLOSE, kernel)

    ret, thresh = cv.threshold(mask_close, 127 ,255, 0)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # print(len(contours))
    
    print(contours)
    if len(contours) == 0:
        print("Red ball wasn't found") 
    cnt = contours[0]
    
    # ((x, y), radius) = cv.minEnclosingCircle(cnt)
    M = cv.moments(cnt)
    cx = int(M['m10'] /  M['m00'])
    cy = int(M['m01'] /  M['m00'])
    if len(cnt) > 0:
        err = 1
        print("Matched")
    else:
        err = None
    
    

    if frame is None:
        return

    return err



def proportional(err):
    global kp
    
    return kp * err    
    


def turnAround():
    wheels[0].setVelocity(velocity * 0.2)
    wheels[1].setVelocity(-velocity * 0.2)


for i in range(2):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))


turnAround()
   

camera = robot.getDevice('cam1')
camera.enable(TIME_STEP)


def process_image():
    cam1 = camera.getImage()
    
    if cam1 is None:
        return None
        
    width = camera.getWidth()
    height = camera.getHeight()
    CHUNK = width*height*4
    cam_int = struct.unpack(str(CHUNK) + 'B', cam1)
    cam_int = np.array(cam_int, dtype=np.uint8).reshape(64, 64, 4)
    cam_int = cam_int[:,:,0:3]
    return cam_int
    


def set_error():
    
    frame = process_image()     
    #cap.read()  
    if frame is None:
        print("N")
        return
    else:
        print("E")       
    return get_error(frame)
                      

def execute_error(error):
    p = proportional(error)
    wheels[0].setVelocity(velocity + p)
    wheels[1].setVelocity(velocity + p) 
    

# while True:
while robot.step(TIME_STEP) != -1:
    r=set_error()
    # print(r)
    if r is None:
        turnAround()
        
    else:
        execute_error(r)
        
