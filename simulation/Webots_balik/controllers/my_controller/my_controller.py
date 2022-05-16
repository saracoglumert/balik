"""my_controller controller."""

# import robot and actuator/sensor code
from controller import Robot, Motor, DistanceSensor, Camera

#DEFINITIONS
cam_sampling_rate=100 #in milliseconds

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


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
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    reading=so[4].getValue()
    image=fisheye.getImage()

    # Process sensor data here.
    print(reading)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    lm.setPosition(10)


def bugnext(robot_coords, target_coords, readings):
    #calculates the next step according to the bug algo. In progress.
    #robot_coords: tuple (x,y)
    #target_coords: tuple (x,y)
    #readings: tuple ((r0,r1,r2,r3,r4,r5,r6,r7),(l0,l1,l2,l3,l4,l5,l6,l7))
    pass
    