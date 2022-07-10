"""camera_calib_controller controller."""

#IMPORTS
from controller import Robot, Motor, DistanceSensor, Camera, GPS
import numpy as np
import cv2

#DEFINITIONS
cam_sampling_rate=1 #in milliseconds
target_coords=(0.8,-0.6)

#FUNCTIONS

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

#flag to determine whether we gave planned motion or not
motion_planned=False
#flag to determine if we rotated correctly
rotated=False
#flag to determine if we moved to target
moved=False

# Main loop:
# - perform simulation step until Webots is stopping the controller
while robot.step(timestep) != -1:
    #### Read the sensors:
    reading=so[4].getValue()
    img=fisheye.getImageArray() #B,G,R,A(?) of each pixel sequentially
    img=np.asarray(img, dtype=np.uint8)
    img=cv2.cvtColor(img,cv2.COLOR_BGRA2RGB)
    img=cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img=cv2.flip(img,1)
    #nparr=np.frombuffer(imagebytes,np.uint8)
    #img_np=cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
    #image=np.array(image)
    #print(imagebytes)
    #### Process sensor data:
    cv2.imwrite("aruco2.jpg",img)
    
    #### Send commands to actuators: