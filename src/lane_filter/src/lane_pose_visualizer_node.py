#!/usr/bin/env python
import rospy
import tf
import numpy as np
from duckietown_msgs.msg import LanePose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class LanePoseVisualzer(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Get vehicle name from namespace
        self.veh_name = rospy.get_namespace().strip("/")
        rospy.loginfo("[%s] Vehicle name: %s" %(self.node_name,self.veh_name))

        # Setup publisher
        self.pub_markers = rospy.Publisher("~lane_pose_markers",MarkerArray,queue_size=1)

        # Setup subscriber
        self.sub_lane_pose = rospy.Subscriber("~lane_pose",LanePose,self.cbLanePose,queue_size=1)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def cbLanePose(self,lane_pose_msg):
        marker_array = MarkerArray()
        # rospy.loginfo("[%s] cbLanePose." %(self.node_name))
        marker_array.markers.append(self.lanePose2Marker(lane_pose_msg))
        self.pub_markers.publish(marker_array)

    def lanePose2Marker(self,lane_pose_msg):
        marker = Marker()
        marker.header.frame_id = self.veh_name
        marker.header.stamp = lane_pose_msg.header.stamp
        marker.ns = self.veh_name + "/lane_pose"
        marker.id = 0
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0.5)
        marker.type = Marker.ARROW
        
        # Get rotation in quaternion
        yaw_quat = tf.transformations.quaternion_about_axis(-lane_pose_msg.phi,[0,0,1])
        # rospy.loginfo("[%s] quat: %s "%(self.node_name,yaw_quat))
        marker.pose.orientation.x = yaw_quat[0]
        marker.pose.orientation.y = yaw_quat[1]
        marker.pose.orientation.z = yaw_quat[2]
        marker.pose.orientation.w = yaw_quat[3]
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = -lane_pose_msg.d
        marker.pose.position.z = 0.0

        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.01

        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.0

        if lane_pose_msg.status == LanePose.NORMAL:
            marker.color.a = 1.0
        else:
            marker.color.a = 0.25

        return marker

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('lane_pose_visualizer_node', anonymous=False)

    # Create the NodeName object
    node = LanePoseVisualzer()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
