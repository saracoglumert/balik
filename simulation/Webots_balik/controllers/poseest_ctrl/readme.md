# Pose Estimator

Estimates aruco marker location.

## Current camera parameter derivation

* take 1600x1600 chessboard photos

* convert to 800x800, use MATLAB to find camera parameters for "Normal" camera model

* convert MATLAB-format params to OpenCV via cameraIntrinsicsToOpenCV

* use K, D matrices in python pose estimator.

The pose estimator seems to find x and y values correctly, but its z guess is double the real value. As a workaround, we halve z in the estimation function.

# TODO:

* Figure out why z estimate is double the real value

* See if function works with a camera resolution of 800x800 (as in without taking it at 1600x1600 and then resizing to 800x800)