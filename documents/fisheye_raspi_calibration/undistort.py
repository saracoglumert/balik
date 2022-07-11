import cv2
import sys
import numpy as np
# You should replace these 3 lines with the output in calibration step
DIM=(1280,720)
#OPENCV results
K=np.array([[1.54694011e+03, 0.0, 5.86248874e+02], [0.0, 1.49820321e+03, 3.31329646e+02], [0.0, 0.0, 1.0]])
D=np.array([[-1.73709447,  5.6209642,  -0.03739107, -0.03543505]])
#MATLAB results
# K=np.array([[656.7987, 0, 586.9032], [0, 656.5124, 276.9949], [0, 0, 1]])
# D=np.array([[-0.2982, 0.0655, 0, 0]])
def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)
