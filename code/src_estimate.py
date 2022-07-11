'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''


import numpy as np
import cv2
import sys
from utils import ARUCO_DICT, aruco_display
import argparse
import time


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)

    result = []
    parameters = cv2.aruco.DetectorParameters_create()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 183, matrix_coefficients,distortion_coefficients)

            temp = [ids[i],rvec,tvec,markerPoints]
            result.append(temp)

            frame = aruco_display(corners,ids,rejected_img_points,frame)
    
    return result,frame

if __name__ == '__main__':
    aruco_dict_type = ARUCO_DICT.get("DICT_5X5_100")
    calibration_matrix_path = "cal_camera.npy"
    distortion_coefficients_path = "cal_dist.npy"
    
    #k = np.load(calibration_matrix_path)
    #k = np.array([[2250,0,1792],[0,2250,1008],[0,0,1]])
    #d = np.load(distortion_coefficients_path)

    #k=np.array([[1.265e3, 0, 1.216e3],[0, 1.259e3, 5.587e2], [0, 0, 1]])
    #d=np.array([[-0.176], [0.786], [-2.215], [2.336]])

    #OPENCV HALF SIZE PARAMS
    #k=np.array([[1.54694011e+03, 0.0, 5.86248874e+02],[0.0, 1.49820321e+03, 3.31329646e+02], [0.0, 0.0, 1.0]])
    #d=np.array([[-1.73709447, 5.6209642, -0.03739107, -0.03543505]])

    #MATLAB HALF SIZE PARAMS
    k=np.array([[656.7987, 0, 586.9032], [0, 656.5124, 276.9949], [0, 0, 1]])
    d=np.array([[-0.2982, 0.0655, 0, 0]])

    print(k)
    print()
    print(d)
    input()

    video = cv2.VideoCapture(0)
    time.sleep(2.0)

    while True:
        ret, frame = video.read()
        
        if not ret:
            break

        frame=cv2.resize(frame,(1280,720))
 
        output,frame = pose_estimation(frame, aruco_dict_type, k, d)
        #cv2.imshow('Estimated Pose', frame)
        try:
            #print("ID  : {}\nRVEC: {}\nTVEC: {}\n\n".format(output[0][0],output[0][1],output[0][2]))
            for i in range(0,len(output)):
                res = output[i][2][0][0]
                resid = output[i][0][0]
                resx = round(res[0],4)
                resy = round(res[1],4)
                resz = round(res[2],4)
                resdist = round(np.linalg.norm(output[i][2]),4)
                print("  ID: {}\n   X: {}\n   Y: {}\n   Z: {}\nDIST: {}\n".format(resid,resx,resy,resz,resdist))
                #print()
                #print(output[0][1])

        except:
            pass

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()
