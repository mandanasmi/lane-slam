# lane-slam
SLAM using line
Tasks to be done: 

1- Estimating the motion of the camera (line based visual odeometery):
  - input: images from duckiebot camera 
  - output: movement estimation with metrics scale

2- build the map by using the lane filter :

we have: 
  - yellow lines 
  - white lines 
 it gives us the position of duckiebot wrt to the yellow center line and the orientation 
 
 
 3 - To get the ground projections: let's say we have a line in an image and project it into real world using the distance (we get from whell velocity command)
 
 4- integrating odeometery 
 
 
 Create the Ros Node, subscribe to wheel commands and gain parameters, 
 based on that how the distance the robot has travalled (x,y, theta)[location, orientation]
 The size of a tile[in the logs] to evalute my estimation 
 (weekend)
 
 
 
