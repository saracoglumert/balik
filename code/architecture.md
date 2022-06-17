*Topics
-map: nxm matrix
-location: current location of robot
-target: target location of robot
-path: required path, from location to target
-voltage: battery voltage
-inmotion: boolean


*Nodes
-Navigation: A* Search using map,location and target, outputs path
-Aruco: Reads aruco markers, outputs location
-Control: Control robot, reads path
-Interaction: Controls robot arm