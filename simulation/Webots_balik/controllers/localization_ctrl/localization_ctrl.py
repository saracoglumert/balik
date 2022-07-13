"""localization_ctrl controller."""

#IMPORTS
from controller import Robot, Motor, DistanceSensor, Camera, GPS
import numpy as np
import cv2
from math import floor

#DEFINITIONS
cam_sampling_rate=1 #in milliseconds
target_coords=(0.8,-0.6)
#fisheye camera params (matlab)
K=np.array([[888.7906, 0, 402.7489], [0, 884.7119, 436.1675], [0, 0, 1]])
D=np.array([[-0.9876], [0.5647], [0.0], [0.0], [0.0]])

### FUNCTIONS
def cam_snap(camera,resolution=(1280,720)):
	"""
	take an image, return opencv-appropriate numpy array

	Parameters
	----------
	camera : webots camera
		the fisheye camera on the robot.
	resolution : tuple (x,y)
		what resolution to resize image to.

	Returns
	-------
	img : numpy array

	"""
	img=camera.getImageArray() #B,G,R,A(?) of each pixel sequentially
	img=np.asarray(img, dtype=np.uint8)
	img=cv2.cvtColor(img,cv2.COLOR_BGRA2RGB)
	img=cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
	img=cv2.flip(img,1)
	img=cv2.resize(img,resolution)
	return img


def cam_estimate_pose(frame, aruco_dict_type=cv2.aruco.DICT_5X5_100, matrix_coefficients=K, distortion_coefficients=D, markersize=0.1):
	"""
	estimate position of a marker wrt the cam

	Parameters
	----------
	frame : image as numpy array
		fisheye camera image
	aruco_dict_type : aruco marker variant, optional
		The default is cv2.aruco.DICT_5X5_100.
	matrix_coefficients : 3x3 matrix, optional
		the K matrix.
	distortion_coefficients : 1x5 matrix, optional
		the D matrix.
	markersize : meters, optional
		size of the markers irl. The default is 0.1.

	Returns
	-------
	results : dict
		{id of detected marker : [rvec, tvec]}

	"""
	results={}
	
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
			# Estimate pose of each marker and return the values rvec and tvec
			rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
				corners[i], markersize, matrix_coefficients, distortion_coefficients
			)
			
			#empirical testing shows that the z axis estimation is double what it should be for some reason.
			#hence divide by two here as a weird fix
			tvec[0][0][2]/=2
			
			#add rvec and tvec under a key
			results[str(ids[i])]=[rvec,tvec]
			
			print(f"tvec: {tvec}, rvec: {rvec} for id {ids[i]}")
			
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
			#draw frame here if you like
	return results

#estimate the pose of the robot from the nearest marker
def rob_estimate_pose(markerlocs):
    # OPTIMIZE: there might a faster way of finding the closest marker
	#determine the closest marker
    closest=[]
	for marker_id in markerlocs:
		#find magnitude
		mag=np.linalg.norm(markerlocs[marker_id][1])
        if len(closest)==0:
            #declare the first id as the closest in the beginning
            closest=[marker_id, markerlocs[marker_id], mag]
        elif mag<closest[2]:
            #found something closer, declare that the closest
            closest=[marker_id, markerlocs[marker_id], mag]
    rvec=closest[1][0]
    tvec=closest[1][1]
	# use tvec and rvec to obtain a transformation matrix
	rotmat,_=cv2.Rodrigues(rvec)
	tvec=tvec[0].transpose()
	RTM=np.hstack((rotmat,tvec))
	RTM=np.vstack((tfmat,np.array([0,0,0,1])))
    #RTM: robot -> marker transformation matrix
    NUM_COLS=3 #TODO: change after hanging markers
    ALPHA=3 #separation in x direction. unit=meters? 
    BETA=3 #separation in y direction. unit=meters?
    #assume not rotated according to room coordinate system
    mx= ALPHA * (marker_id % NUM_COLS)
    my= BETA * floor(marker_id/3)
    #mz=0 since we assume origin is level with ceiling. can be changed.
    mz=0
    OTM=np.array([[1, 0, 0, mx], [0, 1, 0, my], [0, 0, 1, mz], [0, 0, 0, 1]])
    #OTM: origin -> marker transformation matrix
    #now we find OTR using OTM*(RTM)^-1
    OTR=OTM*(np.linalg.inv(RTM))
    #OTR: origin -> robot transformation matrix
	print(OTR)


# camera-based localization function
def cam_localization(camera):
	#take image
	img=cam_snap(camera)
	#run pose estimation on it
	markerlocs=cam_estimate_pose(img)
	#refer to existing table of marker positions to gauge your own position
	return rob_estimate_pose(markerlocs)

# localization function
def localization():
	#if cam sampling rate has passed,
	#take a picture and analyze it.
	#else, get dead reckoning data.
	#add that onto the position estimate to get current pos.
	pass

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
    img=np.asarray(img, dtype=np.uint8)
    img=cv2.cvtColor(img,cv2.COLOR_BGRA2RGB)
    img=cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img=cv2.flip(img,1)
    img=cv2.resize(img,(800,800))
    #### Process sensor data:
    output=pose_estimation(img, aruco_dict_type, K, D)
    
    #### Send commands to actuators: