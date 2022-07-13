"""poseest_ctrl controller."""

#IMPORTS
from controller import Robot, Motor, DistanceSensor, Camera, GPS
import numpy as np
import cv2

#DEFINITIONS
cam_sampling_rate=1 #in milliseconds

#FUNCTIONS
def pose_estimation(
	frame, aruco_dict_type, matrix_coefficients, distortion_coefficients
):
	"""
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    """

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
	parameters = cv2.aruco.DetectorParameters_create()

	corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
		gray,
		cv2.aruco_dict,
		parameters=parameters,
		cameraMatrix=matrix_coefficients,
		distCoeff=distortion_coefficients,
	)

	# If markers are detected
	if len(corners) > 0:
		for i in range(0, len(ids)):
			# Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
			# size of marker is 10cmx10cm, so 0.1 below
			rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
				corners[i], 0.1, matrix_coefficients, distortion_coefficients
			)
			# Draw a square around the markers
			cv2.aruco.drawDetectedMarkers(frame, corners)

			# Draw Axis
			cv2.aruco.drawAxis(
				frame,
				matrix_coefficients,
				distortion_coefficients,
				rvec,
				tvec,
				0.01,
			)
			#empirical testing shows that the z axis estimation is double what it should be for some reason.
			#hence divide by two here as a weird fix
			tvec[0][0][2]/=2
			print(f"tvec: {tvec}, rvec: {rvec}")

	return frame

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

aruco_dict_type = cv2.aruco.DICT_5X5_100
#camera params (matlab)
K=np.array([[888.7906, 0, 402.7489], [0, 884.7119, 436.1675], [0, 0, 1]])
D=np.array([[-0.9876], [0.5647], [0.0], [0.0], [0.0]])

# Main loop:
# - perform simulation step until Webots is stopping the controller
while robot.step(timestep) != -1:
    #### Read the sensors:
    reading=so[4].getValue()
    img=fisheye.getImageArray() #B,G,R,A(?) of each pixel sequentially
	#format image to use with opencv
    img=np.asarray(img, dtype=np.uint8)
    img=cv2.cvtColor(img,cv2.COLOR_BGRA2RGB)
    img=cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img=cv2.flip(img,1)
    img=cv2.resize(img,(800,800))
    #### Process sensor data:
    output=pose_estimation(img, aruco_dict_type, K, D)
    
    #### Send commands to actuators: