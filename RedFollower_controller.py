"""PID controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
from controller import Robot
import numpy as np
import argparse
import cv2
import math
import struct
from controller import *

TIME_STEP = 64
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




def get_error(frame):
    int err
    err=0

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
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))


turnAround()
   

camera = robot.getCamera('cam1')
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
    
    frame = process_image()     #cap.read()     
    if frame is None:
        return
    return get_error(frame)
                      

def execute_error(error):
    p = proportional(error)
    wheels[0].setVelocity(velocity - p)
    wheels[1].setVelocity(velocity + p) 
    


while robot.step(TIME_STEP) != -1:
    r=set_error()
    
    if r is None:
        turnAround()
        
    else:
        execute_error(r)
        
