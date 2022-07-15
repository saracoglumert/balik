# Localization Algorithm

### idea:
We can find a mostly accurate location + orientation if we can detect a marker. We also have
a good estimate of where we are going from our dead reckoning/odometry stuff. Take camera as
ground truth whenever possible, use odometry in the day-to-day to fill in the blanks.

The "good" way of doing this is using a Kalman filter and representing the location of the robot
with a probability density function, but if this idea works, no need.

If the output is too noisy, try low-pass filter maybe?

### algo:
```
loop:
	if new camera image:
		if marker in image:
			location = pose_est()
	else:
		location += odometry_delta
```

## Finding robot position if we know the marker position wrt robot

We need to know the marker position wrt the world coords to find the robot pose. Say that we arranged the markers like this:
```
    x
 ┌─────►
 │
y│  ┌──────────────────────────────────────────────────────────────────┐
 │  │               ▲                                                  │
 ▼  │               │                                                  │
    │               │b                                                 │
    │               │                                                  │
    │               ▼                                                  │
    │               0                1               2                 │
    │◄─────────────►▲◄──────────────►                                  │
    │       a       │        a                                        ┌┼┐
    │               │b                                                │┼│
    │               │                                                 │┼│door
    │               ▼                                                 │┼│
    │               3                4               5                │┼│
    │                                                                 └┼┘
    │                                                                  │

```
now the position of each marker is defined by $x = a * ((id)mod3 + 1) + offset$ and $y = b * (floor(\dfrac{id}{3} + 1) + offset$. Also assume that each marker has the same orientation.

So now we know ${}^OT_{M_1}$, and we get ${}^RT_{M_1}$ from the robot's pose. Then we can find the robot's pose using ${}^OT_R = {}^OT_{M_1} ({}^RT_{M_1})^{-1}$.

Note: We have to reflect the ${}^RT_{M_1}$ matrix on all axes using the following expression before we do the above calculation (probably due to the camera mirroring the image):
```
     [-1  0  0  0]
 M=M*[ 0 -1  0  0]
     [ 0  0 -1  0]
     [ 0  0  0  1]
```

If we have multiple markers to reference, we might encounter problems, so for now we will find the closest marker to the robot and use that to find pose. Later, this part can be changed to a Kalman filter.

OpenCV returns a tvec and an rvec, where rvec needs to be converted into a rotation matrix using cv2.Rodrigues. tvec is the distance of the marker to the camera.

Note: in our case we may change cx cy to (width-1)/2 and (height-1)/2 as it seems to have the wrong offset.

## Other notes:

lab height (floor to aruco-brackets)=330.5cm 
aruco bracket length: in x: 140cm, in y: 140cm

sim lab origin: 5.49, 3.72
