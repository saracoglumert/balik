# -*- coding: utf-8 -*-
"""
locl - localization-related functions
Created on Tue Jul 12 15:26:47 2022

@author: Anil
"""
import numpy as np
import cv2
from math import floor, sin, cos, asin

### CONSTANTS
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


def cam_estimate_pose(frame, aruco_dict_type=cv2.aruco.DICT_5X5_100, matrix_coefficients=K, distortion_coefficients=D, markersize=0.5):
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
		size of the markers irl. The default is 0.1 for irl, 0.5 for sim.

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
	else:
		return None

def rob_estimate_pose(markerlocs):
	"""
	estimate the pose of the robot from the nearest marker.
	Multiple marker support can be added in some ways:
		- average of all marker locs
		- kalman filter for probabilistic estimate

	Parameters
	----------
	markerlocs : dict
		rel. transforms of markers. {"id":[rvec,tvec]}

	Returns
	-------
	OTR : 4x4 matrix
		transformation matrix for the robot's position.

	"""
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
	RTM=np.vstack((RTM,np.array([0,0,0,1])))
	#RTM: robot -> marker transformation matrix
	#TODO: change params below after hanging markers
	NUM_COLS=3
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
	return OTR


def cam_localization(camera):
	"""
	camera-based localization function

	Parameters
	----------
	camera : webots camera object
		the fisheye camera on the robot.

	Returns
	-------
	robot_pose: 4x4 transformation matrix
		the robot's pose wrt the lab's origin.
		if no new markers were detected,
		returns None.
	"""
	#take image
	img=cam_snap(camera)
	#run pose estimation on it
	markerlocs=cam_estimate_pose(img)
	if markerlocs != None:
		#if any marker was detected
		#refer to existing table of marker positions to gauge your own position
		return rob_estimate_pose(markerlocs)
	else:
		return None

def dr_reset():
	"""
	set the robot's internal, dead-reckoning based location estimate to zero.
	for the irl implementation, send a SETO command (command #7).

	Returns
	-------
	None.

	"""
	global internal_dr
	internal_dr=np.identity(4)

def dr_differentiate(lme,rme):
	"""
	for robot internal dead-reckoning simulation
	find theta\dot of the two wheels by differentiating
	encoder readouts.

	Parameters
	----------
	lme : webots positionsensor
		left wheel.
	rme : webots positionsensor
		right wheel.

	Returns
	-------
	[lmedot, rmedot, sampling]
		lmedot,rmedot: change in wheel angle. in radians.
		sampling: time delta

	"""
	global internal_lme_prev
	global internal_rme_prev
	
	lmeval=lme.getValue()
	rmeval=rme.getValue()
	sampling=lme.getSamplingPeriod() #sampling rate, time delta
	#differentiate
	lmedot=(lmeval-internal_lme_prev)/sampling
	rmedot=(rmeval-internal_rme_prev)/sampling
	#set prev values
	internal_lme_prev=lmeval
	internal_rme_prev=rmeval
	return [lmedot, rmedot, sampling]
	
#dead reckoning localization. Normally the robot should compute this part.
#I will use the unicycle model to model the robot.
#xdot=1/2*(vr+vl)*cos(theta)
#ydot=1/2*(vr+vl)*sin(theta)
#thetadot=(vr-vl)/(328mm)
def dr_localization(lmedot, rmedot, sampling):
	global internal_dr
	WHEEL_RADIUS=97.5 #mm
	WHEEL_SEPARATION=328 #mm
	
	theta=asin(internal_dr[1][0]) #find theta from the rot. mat.
	vr=rmedot*WHEEL_RADIUS
	vl=lmedot*WHEEL_RADIUS
	
	xdot=1/2*(vr+vl)*cos(theta)
	ydot=1/2*(vr+vl)*sin(theta)
	thetadot=(vr-vl)/(WHEEL_SEPARATION)
	
	#multiply by time delta to find the incremental change
	xdelta=xdot*sampling
	ydelta=ydot*sampling
	thetadelta=thetadot*sampling
	
	#construct transformation matrix
	#actually a 2d tf matrix but we represent it as 3d
	delta_tfmat=np.array([[cos(thetadelta), -sin(thetadelta), 0, xdelta],
						  [sin(thetadelta),  cos(thetadelta), 0, ydelta],
						  [              0,                0, 1,      0],
						  [              0,                0, 0,      0]])
	#add the newly calculated delta to the internal transform.
	internal_dr=internal_dr*delta_tfmat


def dr_getestimate():
	"""
	Simulates getting the dead-reckoning based
	position estimate from the robot internals.

	Returns
	-------
	internal_dr : 4x4 transformation matrix

	"""
	global internal_dr
	return internal_dr

# count how many main loops have passed.
# to determine when to snap a new fisheye pic.
tick_counter=0
#variable to act as the internal dead-reckoning based position estimate
#of the robot.
internal_dr=None
#previous value of encoders for differentiation to simulate robot's internal
#dead reckoning systems
internal_lme_prev=None
internal_rme_prev=None

# localization function.
# check camera occasionally. if any markers are detected,
# use location from the markers as ground truth.
# add the dead reckoning delta to the marker-derived location
# until a new marker is detected.
def localization(fisheye_cam, lme, rme, marker_pos):
	##simulate robot's internal dead-reckoning system
	#normally done by the robot.
	[lmedot, rmedot, sampling]=dr_differentiate(lme, rme)
	dr_localization(lmedot, rmedot, sampling)
	##
	
	#if cam sampling rate has passed,
	if tick_counter>=100:	
		#take a picture and analyze it.
		marker_pos_temp=cam_localization(fisheye_cam)
		if marker_pos_temp != None:
			marker_pos=marker_pos_temp
			#reset the dead reckoning estimate to get the delta from
			#this new marker position
			dr_reset()
	else:
		#else, get dead reckoning data.
		dr_delta=dr_getestimate()
		#add that onto the last marker position estimate to get current pos.
		current_pos=marker_pos+dr_delta
	return [current_pos, marker_pos]
