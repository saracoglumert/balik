## Topics
- **map**: nxm matrix
- **location**: current location of robot
- **target**: target location of robot
- **path**: required path, from location to target
- **voltage**: battery voltage
- **inmotion**: boolean


## Nodes
- **navigation**: A* Search using map,location and target, outputs path
- **aruco**: Reads aruco markers, outputs location
- **control**: Control robot, reads path
- **arm**: Controls robot arm
- **interaction**: Twitter etc.
- **charge**: Automatic charging
- **sensor**: Sensor readings (ultrasonic) 