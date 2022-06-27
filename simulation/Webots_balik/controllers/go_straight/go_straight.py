"""go_straight controller."""

#IMPORTS
from controller import Robot, Motor, DistanceSensor, Camera, GPS
import numpy as np
from math import sqrt, pi, acos, nan, atan2

#DEFINITIONS
cam_sampling_rate=1 #in milliseconds
target_coords=(0.8,-0.6)

#FUNCTIONS
def dotProd(vec1,vec2):
    '''
    dot product. give vectors of same length.
    '''
    acc=0
    for dim in range(0,len(vec1)):
        acc+=vec1[dim]*vec2[dim]
    return acc
    
def findsin(vec1,vec2):
    v1=np.array(vec1)
    v2=np.array(vec2)
    cprod=np.cross(v1,v2)
    m1=np.linalg.norm(v1)
    m2=np.linalg.norm(v2)
    return cprod/(m1*m2)

def robFwdP(diff,lm,rm):
    '''
    move robot forward by diff. returns true if fully moved.
    '''
    kp=2
    thres=0.1
    e=diff
    if abs(e)>thres:
        lm.setVelocity(kp*e+1)
        rm.setVelocity(kp*e+1)
        return False
    else:
        print("moved")
        lm.setPosition(float('inf'))
        rm.setPosition(float('inf'))
        lm.setVelocity(0)
        rm.setVelocity(0)
        return True

def robRotP(diff,lm,rm):
    '''
    rotate robot CCW by ang rad. returns true if fully rotated.
    '''
    kp=7
    thres=0.01
    e=diff
    if abs(e)>thres:
        lm.setVelocity(-kp*e)
        rm.setVelocity(kp*e)
        return False
    else:
        print("rotated")
        lm.setPosition(float('inf'))
        rm.setPosition(float('inf'))
        lm.setVelocity(0)
        rm.setVelocity(0)
        return True
    

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
    
    
    #dot product to find cos(angle) between our heading and trajectory
    cosang=(dotProd(heading, traj_dir))
    #cross product to find sin(angle)
    sinang=findsin(heading,traj_dir)
    ang=atan2(sinang,cosang)
    
    
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
    image=fisheye.getImage() #B,G,R,A of each pixel sequentially
    location=gps.getValues()

    #### Process sensor data:
    heading=getHeading(gps,gpsf)
    
    #### Send commands to actuators:
    if sum(heading)!=0:
        if not motion_planned:
            plan=goStraight(location[0:2], target_coords, heading) #get how much to rotate and how much to go
            #print(plan)
            lm.setPosition(float('inf'))
            rm.setPosition(float('inf'))
            motion_planned=True
         
        elif not rotated:
            #get current angle diff
            current=goStraight(location[0:2], target_coords, heading)
            
            #print(f"LOCATION: {location[0:2]} HEADING: {heading}")
            print(f"rANG DELTA: {current[0]} \t\t\t  COORD DELTA: {current[1]}")
            
            #call rotation controller
            res=robRotP(current[0],lm,rm)
            if res:
                rotated=True
                #if within angle thres, start going forward
        elif not moved:
            #get current coord diff
            current=goStraight(location[0:2], target_coords, heading)
            
            #print(f"LOCATION: {location[0:2]} HEADING: {heading}")
            print(f"mANG DELTA: {current[0]} \t\t\t  COORD DELTA: {current[1]}")
            
            #call forward motion controller
            res=robFwdP(current[1],lm,rm)
            if res:
                moved=True
                #if within coord thres, print that the thing is complete!
                print("COMPLETED")