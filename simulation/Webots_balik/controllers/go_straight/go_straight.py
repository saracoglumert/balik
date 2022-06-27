"""go_straight controller."""

#IMPORTS
from controller import Robot, Motor, DistanceSensor, Camera, GPS
import numpy as np
from math import sqrt, pi, acos, nan

#DEFINITIONS
cam_sampling_rate=100 #in milliseconds
target_coords=(0.5,0.1)

#FUNCTIONS
def dotProd(vec1,vec2):
    '''
    dot product. give vectors of same length.
    '''
    acc=0
    for dim in range(0,len(vec1)):
        acc+=vec1[dim]*vec2[dim]
    return acc

def robFwd(dist,lm,rm):
    '''
    move the robot forward by dist meters.
    wheels have d=195mm -> r=97.5mm
    '''
    pos=dist/0.0975
    lm.setPosition(lm.getTargetPosition()+pos)
    rm.setPosition(rm.getTargetPosition()+pos)

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
    lm.setPosition(lm.getTargetPosition()-pos)
    rm.setPosition(rm.getTargetPosition()+pos)
    

def goStraight(robot_coords, target_coords, heading):
    '''
    IN PROGRESS
    calculates angle and magnitude to go to a point
    robot_coords: tuple (x,y)
    target_coords: tuple (x,y)
    heading: tuple (x,y)
    '''
    traj_vec=[target_coords[0]-robot_coords[0], target_coords[1]-robot_coords[1]] #draw a straight line from start to end
    magnitude=(sqrt(traj_vec[0]**2 + traj_vec[1]**2))
    traj_dir=[traj_vec[0]/magnitude, traj_vec[1]/magnitude] #normalize the vector to get direction
    
    #dot product to find angle between our heading and trajectory
    ang=acos(dotProd(heading, traj_dir))
    
    #return instructions
    return [ang,magnitude]
    
def getHeading(gps,gpsf):
    '''
    Get orientation of robot using the two test gps modules.
    '''
    vec_start=gps.getValues()
    vec_end=gpsf.getValues()
    
    vec=[vec_end[0]-vec_start[0], vec_end[1]-vec_start[1]]
    magnitude=(sqrt(vec[0]**2+vec[1]**2))

    return [vec[0]/magnitude, vec[1]/magnitude] #return it normalized

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

#flag to determine whether we gave motion instructions or not
motion_planned=False

# Main loop:
# - perform simulation step until Webots is stopping the controller
while robot.step(timestep) != -1:
    #### Read the sensors:
    reading=so[4].getValue()
    image=fisheye.getImage() #B,G,R,A of each pixel sequentially
    location=gps.getValues()

    #### Process sensor data:
    heading=getHeading(gps,gpsf)
    print(f"LOCATION: {location[0:2]} HEADING: {heading}")
    if not motion_planned and sum(heading)>0:
        plan=goStraight(location[0:2], target_coords, heading) #get how much to rotate and how much to go
        #execute those
        #potential pitfall: is angle output as CCW?
        robRot(plan[0],lm,rm)
        robFwd(plan[1],lm,rm)
        motion_planned=True
    
    #### Send commands to actuators:
    