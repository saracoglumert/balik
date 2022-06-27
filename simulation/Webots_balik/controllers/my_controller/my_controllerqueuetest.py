"""my_controller controller."""

#IMPORTS
from controller import Robot, Motor, DistanceSensor, Camera, GPS
import numpy as np
from cv2 import aruco
from math import sqrt, pi

#DEFINITIONS
cam_sampling_rate=100 #in milliseconds
target_coords=(1,1)

#FUNCTIONS
def findAruco(img):
    '''
    test function to detect arucomarkers.
    '''
    return aruco.detectMarkers(img,aruco.DICT_5X5_100)

def robFwd(dist,lm,rm):
    '''
    move the robot forward by dist meters.
    wheels have d=195mm -> r=97.5mm
    '''
    pos=dist/0.0975
    lm.setPosition(pos)
    rm.setPosition(pos)

def robRot(angle,lm,rm):
    '''
    rotate the robot by angle CCW in radians.
    wheels have d=195mm -> r=97.5mm
    wheel separation is roughly 328mm -> r_o=165mm
    calculations yielded wrong results, so coefficient found experimentally.
    '''
    #pos=(0.164/0.0975)*angle
    #pos=2*angle
    #pos=(0.165/0.0975)*angle
    pos=1.86*angle #why does this work? idk. determined experimentally.
    lm.setPosition(-pos)
    rm.setPosition(pos)
    

def bugNext(robot_coords, target_coords, heading, readings):
    '''
    IN PROGRESS
    calculates the next step according to the bug algo.
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
 
#CLASSES
class Queue(cmdlist):
     '''
     Queue of actions to be followed by robot.
     Can be paused/resumed.
     TODO: figure out how to determine if a command is done:
         -wait a predetermined amount?
         -feedback? (may require movement function rewrite)
     '''
     def __init__(self, cmdlist):
         '''
         cmdlist:list of commands to be followed.
                 each command is a list: [cmd,val]
                 where cmd is either 'r' for rotate or 'f' for forward
                 and val is CCW radians of rotation or meters of distance
         '''
         self.cmdlist=cmdlist
     
     def addcmd(self, cmd):
         '''
         add a command to the end of the queue.
         '''
         self.cmdlist.append(cmd)
     #TODO write play/resume functions after figuring out behavior

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
# - perform simulation step until Webots is stopping the controller
while robot.step(timestep) != -1:
    #### Read the sensors:
    reading=so[4].getValue()
    image=fisheye.getImage() #B,G,R,A of each pixel sequentially
    location=gps.getValues()

    #### Process sensor data:
    #heading=getHeading(gps,gpsf)
    #print(f"HEADING: {heading}")
    
    #print(image)
    #imgnp=np.asarray(image)
    #print(findaruco(imgnp))
    #orientation=getHeading(gps,gpsf)
    #print(orientation)
    
    
    #### Send commands to actuators:
    #lm.setPosition(3.14)
    #rm.setPosition(3.14)
    robRot(2*pi,lm,rm)
    robFwd(100,lm,rm)