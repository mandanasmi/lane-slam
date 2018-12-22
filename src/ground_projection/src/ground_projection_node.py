#!/usr/bin/env python

import rospy
import cv2
from ground_projection.srv import EstimateHomography, EstimateHomographyResponse, GetGroundCoord, GetGroundCoordResponse, GetImageCoord, GetImageCoordResponse
from duckietown_msgs.msg import (Pixel, Vector2D, Segment, SegmentList)
from sensor_msgs.msg import (Image, CameraInfo)
from cv_bridge import CvBridge
import numpy as np
from ground_projection.GroundProjection import GroundProjection


class GroundProjectionNode(object):

    def __init__(self):
        self.node_name="Ground Projection"
        self.active = True
        self.bridge=CvBridge()

        self.robot_name = rospy.get_param("~config_file_name","robot_not_specified")
        self.gp = GroundProjection(self.robot_name)

        camera_info_topic = "/"+self.robot_name+"/camera_node/camera_info"
        rospy.loginfo("camera info topic is " + camera_info_topic)
        rospy.loginfo("waiting for camera info")
        camera_info = rospy.wait_for_message(camera_info_topic,CameraInfo)
        rospy.loginfo("camera info received")

        self.gp.initialize_pinhole_camera_model(camera_info)
        # Params


        self.gp.robot_name = self.robot_name
        self.gp.rectified_input_ = rospy.get_param("rectified_input", False)
        self.image_channel_name = "image_raw"

        # Subs and Pubs
        self.pub_lineseglist_ = rospy.Publisher("~lineseglist_out",SegmentList, queue_size=1)
        self.sub_lineseglist_ = rospy.Subscriber("~lineseglist_in",SegmentList, self.lineseglist_cb)


        # TODO prepare services
        self.service_homog_ = rospy.Service("~estimate_homography", EstimateHomography, self.estimate_homography_cb)
        self.service_gnd_coord_ = rospy.Service("~get_ground_coordinate", GetGroundCoord, self.get_ground_coordinate_cb)
        self.service_img_coord_ = rospy.Service("~get_image_coordinate", GetImageCoord, self.get_image_coordinate_cb)


    def rectifyImage(self,img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="mono8")
        except CvBridgeError as e:
            logger.error(e)
        return gp.rectify(cv_image)

    def lineseglist_cb(self,seglist_msg):
        seglist_out = SegmentList()
        seglist_out.header = seglist_msg.header
        for received_segment in seglist_msg.segments:
            new_segment = Segment()
            new_segment.points[0] = self.gp.vector2ground(received_segment.pixels_normalized[0])
            new_segment.points[1] = self.gp.vector2ground(received_segment.pixels_normalized[1])
            new_segment.color = received_segment.color
            # TODO what about normal and points
            seglist_out.segments.append(new_segment)
        self.pub_lineseglist_.publish(seglist_out)

    def get_ground_coordinate_cb(self,req):
        return GetGroundCoordResponse(self.gp.pixel2ground(req.normalized_uv))

    def get_image_coordinate_cb(self,req):
        return GetImageCoordResponse(self.gp.ground2pixel(req.gp))

    def estimate_homography_cb(self,req):
        rospy.loginfo("Estimating homography")
        rospy.loginfo("Waiting for raw image")
        img_msg = rospy.wait_for_message("/"+self.robot_name+"/camera_node/image/raw",Image)
        rospy.loginfo("Got raw image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.gp.estimate_homography(cv_image)
        rospy.loginfo("wrote homography")
        return EstimateHomographyResponse()

    def onShutdown(self):
        rospy.loginfo("[GroundProjectionNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('ground_projection',anonymous=False)
    ground_projection_node = GroundProjectionNode()
    rospy.on_shutdown(ground_projection_node.onShutdown)
    rospy.spin()
