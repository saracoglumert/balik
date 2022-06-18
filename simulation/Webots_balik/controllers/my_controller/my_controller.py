"""my_controller controller."""

#IMPORTS
from controller import Robot, Motor, DistanceSensor, Camera, GPS
import numpy as np
from cv2 import aruco
from math import sqrt

#DEFINITIONS
cam_sampling_rate=100 #in milliseconds
target_coords=(1,1)

#FUNCTIONS
def findaruco(img):
    return aruco.detectMarkers(img,aruco.DICT_5X5_100)

def bugnext(robot_coords, target_coords, heading, readings):
    '''
    calculates the next step according to the bug algo. In progress.
    robot_coords: tuple (x,y)
    target_coords: tuple (x,y)
    heading: tuple (x,y)
    readings: tuple ((f0,f1,f2,f3,f4,f5,f6,f7),(b0,b1,b2,b3,b4,b5,b6,b7))
    '''
    
    pass
    
def getHeading(gps,gpsf):
    '''
    Get orientation of robot using the two test gps modules.
    '''
    vec_start=gps.getValues()
    vec_end=gpsf.getValues()
    vec=[vec_end[0]-vec_start[0], vec_end[1]-vec_start[1]]
    magnitude=(sqrt(vec[0]**2+vec[1]**2))
    return [vec[0]/magnitude, vec[1]/magnitude]
    

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

#initialize motors
lm=robot.getDevice("left wheel")
rm=robot.getDevice("right wheel")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    #### Read the sensors:
    reading=so[4].getValue()
    image=fisheye.getImage() #B,G,R,A of each pixel sequentially
    location=gps.getValues()

    #### Process sensor data:
    print(getHeading(gps,gpsf))
    #print(image)
    #imgnp=np.asarray(image)
    #print(findaruco(imgnp))
    #orientation=getHeading(gps,gpsf)
    #print(orientation)
    
    
    #### Send commands to actuators:
    lm.setPosition(100)
    rm.setPosition(100)




    