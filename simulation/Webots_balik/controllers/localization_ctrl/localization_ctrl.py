"""localization_ctrl controller."""

#IMPORTS
from controller import Robot, Motor, DistanceSensor, Camera, GPS, PositionSensor
import numpy as np
import cv2
from math import floor
import locl

#DEFINITIONS
cam_sampling_rate=1 #in milliseconds
target_coords=(0.8,-0.6)
#fisheye camera params (matlab)
K=np.array([[888.7906, 0, 402.7489], [0, 884.7119, 436.1675], [0, 0, 1]])
D=np.array([[-0.9876], [0.5647], [0.0], [0.0], [0.0]])

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#initialize GPS. This will act as a placeholder source of location data
#until the cam-based localization is implemented.
gps=robot.getDevice("gps")
gps.enable(cam_sampling_rate) #same as cam to simulate camera localization speed
gpsf=robot.getDevice("gpsf")
gpsf.enable(cam_sampling_rate)

#initialize fisheye camera
fisheye=robot.getDevice("fisheye")
fisheye.enable(cam_sampling_rate)

#initialize sonar.
#angle between two consecutive sensor directions is 20 degrees except for the four side sensors (so0, so7, so8 and so15) for which the angle is 40 degrees.
so=[]
soNames=[
    'so0', 'so1', 'so2',  'so3',  'so4',  'so5',  'so6',  'so7', #front
    'so8', 'so9', 'so10', 'so11', 'so12', 'so13', 'so14', 'so15' #back
    ]	

for soName in soNames:
    so.append(robot.getDevice(soName))
    so[-1].enable(timestep)

#initialize motor encoders
lme=robot.getDevice("left wheel sensor")
lme.enable(1)
rme=robot.getDevice("right wheel sensor")
rme.enable(1)

#initialize motors
lm=robot.getDevice("left wheel")
rm=robot.getDevice("right wheel")


# Main loop:
# - perform simulation step until Webots is stopping the controller
while robot.step(timestep) != -1:
    #### Read the sensors:
    pic=locl.cam_snap(fisheye)
    #### Process sensor data:
    print(locl.cam_estimate_pose(pic))
    #### Send commands to actuators: