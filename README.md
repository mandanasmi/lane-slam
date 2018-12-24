# Lane-based SLAM


This project is done as a course project for Duckietown course in Fall 2018/ Winter 2019.

The goal of the project is building a map of duckietown streets by associating semantic to the roads and representing the environment in terms of lines and their colors. We're classifying line to 3 different categories: 
  *Yellow lines: the line in the center of the road
  *White lines: the lines in the sides of the road
  *Red lines: showing stop lines
 We use a different line detector that provide descriptors rather that the one from Duckietown software stack. 
To achieve this map, we defined several individual packages that are integrated using ROS. 

By following the instruction, you should be able to see the following map visualizations constructed of colored lines from images in the log you're experimenting. You can visualize the line-based semantic map built by your duckiebot using rviz.

<table align="center">
  <tr>
     <td>
 		<img style='width:15em; height:15em' src="figures/map1.png"/>
	    <p>Map after X seconds</p>
     </td>
     <td>
	    <img style='width:15em; height:15em' src="figures/map2.png"/>
	    <p>Map after Y seconds (Y > X)</p>
     </td>
     <td>
	    <img style='width:15em; height:15em' src="figures/map3.png"/>
	    <p>Map after Z seconds (Y > X and Z >> X)</p>
     </td>
  </tr>
</table>

## System Requirements: 

As mentioned, we build a map on a log of a duckiebot. To run the following demo, you'll need the actual log (a `.bag` file), in which the image seen by the robot is recorded (probably using the `ros-picam` node), as well as some source of odometry (in our code we use published executed wheel commands, but you could use any other odometry method).

You'll also need a Linux computer with ROS. If you're using a mac OSx system you probably need to install a linux on a virtual machine as you need to have ROS installed. Then clone the repo and you'll have the ROS workspace containing all packages required. You can build that workspace and source before trying to run anything.

# Modules in the Lane-based SLAM

Here are the modules we defined to build the semantic map: 

## Line Detector

This module takes an image as an input and outputs detected lines. In this package, we made some enhancement on the 'line-detector' from the Duckietown stack. We use a new line segment detector [LSD](https://docs.opencv.org/3.4/db/d73/classcv_1_1LineSegmentDetector.html). This detector gets more stable and longer line segments compared to `HoughLines`, which seems to be the default. 

To run this node using `roslaunch`, execute the following command.
`roslaunch line_detector line_detector_node.launch veh:=neo local:=true` (on your laptop)  

## Line Descriptor

It takes line segments from the detector and produces descriptors. This package uses _OpenCV_ functions to compute binary descriptors for a bunch of line segments, to help in matching/associating lines. Currently, this functionality is in beta, but people are welcome to play around with the code in here.  

## Ground Projection

This package takes in a list of line segments detected in the image and projects them onto the 3D ground plane using a homography matrix computed based on the extrinsic calibration parameters.  

This node will publish a message of type `SegmentList`, which contains a list of _ground-projected_ line segments (i.e., line segments on the 3D ground plane), with a topic name `/neo/ground_projection/lineseglist_out_lsd` (assuming *neo* is the name of the Duckiebot)  


## Line sanity
This package takes in _ground-projected_ line segments and _filters out_ spurious lines. It subscribes to a topic that publishes a `SegmentList` message type, applies filters, and publishes the filtered line segments to another topic `filtered_segments_lsd` (again, as a `SegmentList` message type). 

*Types of spurious lines filtered out:*

1. Lines behind the robot  
2. Lines that are not white or yellow, and cannot be confidently classified as being left or right edges of a white or yellow line (we ignore RED lines for now).  
3. Lines that are farther ahead from the robot, above a certain distance threshold.  
4. All lines that do not satisfy a certain angular threshold.  


## Odometry

This package takes an input from the logs (in our case wheel commands) and produces an estimate of the duckiebot's position.  


## Show Map

As the last step, we Combine odometry and line detections to build a map. It takes the estimated position from odometry and the ground projected lines to build an display a map of the lines. 

## Running All together

We have an individual launch file for each module, so you can run each node separated from others and then launch show_map to see the visualization or you can just run the global launch file that launch every node themselves in the right order. 




 
 
 
