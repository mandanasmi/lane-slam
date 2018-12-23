#!/usr/bin/env python
from duckietown_msgs.msg import (AntiInstagramTransform, BoolStamped, Segment,
    SegmentList, Vector2D)
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker
import rospy
import threading
import time
import numpy as np


class LineSanityNode(object):

    def __init__(self):

        self.lanewidth = 0.23
        self.linewidth_white = 0.05
        self.linewidth_yellow = 0.025
        self.d_min = -0.15
        self.d_max = 0.3
        self.phi_min = -1.5
        self.phi_max = 1.5

        self.node_name = "LineSanityNode"

        self.vehicle = rospy.get_param('~vehicle')

        # Subscribers
        self.subscriber = rospy.Subscriber("/"+self.vehicle+"/ground_projection/lineseglist_out_lsd", SegmentList, self.cbSegmentList, queue_size=1)
        # Publishers
        self.publisher = rospy.Publisher("~filtered_segments_lsd", SegmentList, queue_size=1)
        

        rospy.loginfo("[%s] Vehicle name = %s." % (self.node_name, self.vehicle))
        rospy.loginfo("[%s] Initialized." %(self.node_name))
        rospy.loginfo("[%s] %s/ground_projection/lineseglist_out_lsd" % (self.node_name, self.vehicle))


    def cbSegmentList(self, segmentList):
        # print('Processing segment list')
        # Start a daemon thread to process the segment list.
        filteredSegments = self.processSegmentList(segmentList)
        print('Filtered segments:', str(len(filteredSegments.segments)) + '/' + str(len(segmentList.segments)))
        self.publisher.publish(filteredSegments)


    def processSegmentList(self,  segmentList):
        
        filteredSegments = SegmentList()
        filteredSegments.header = segmentList.header
        for segment in segmentList.segments:
            # Filter off segments behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue
            # Discard RED lines, for now
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            # Apply a few more fancy filters to the segment
            d_i, phi_i, l_i, state = self.fancyFilters(segment)
            # Ignore if the line segment is UNKNOWN (i.e., = 0)
            if state == 0:
                continue
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i > self.phi_max:
                continue

            # filteredSegments.append(segment)
            # print(filteredSegments)
            filteredSegments.segments.append(segment)
        
        return filteredSegments


    def fancyFilters(self, segment):
        # States of a line segment
        # 0 -> UNKNOWN, 1 -> right edge of white lane, 2 -> left edge of white lane, 
        # 3 -> left edge of yellow lane, 4 -> right edge of yellow lane, 
        # 5 -> red line
        state = 0

        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2-p1)/np.linalg.norm(p2-p1)
        n_hat = np.array([-t_hat[1],t_hat[0]])
        d1 = np.inner(n_hat,p1)
        d2 = np.inner(n_hat,p2)
        l1 = np.inner(t_hat,p1)
        l2 = np.inner(t_hat,p2)
        if (l1 < 0):
            l1 = -l1;
        if (l2 < 0):
            l2 = -l2;
        l_i = (l1+l2)/2
        d_i = (d1+d2)/2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == segment.WHITE: # right lane is white
            if(p1[0] > p2[0]): # right edge of white lane
                d_i = d_i - self.linewidth_white
                state = 1
            else: # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
                state = 2
            d_i = d_i - self.lanewidth/2

        elif segment.color == segment.YELLOW: # left lane is yellow
            if (p2[0] > p1[0]): # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
                state = 3
            else: # right edge of yellow lane
                d_i = -d_i
                state = 4
            d_i =  self.lanewidth/2 - d_i

        return d_i, phi_i, l_i, state


    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x)/2
        y_c = (segment.points[0].y + segment.points[1].y)/2
        return sqrt(x_c**2 + y_c**2)


    def onShutdown(self):
        rospy.loginfo("[LineSanityNode] Shutdown.")



if __name__ == '__main__': 
    rospy.init_node('line_sanity',anonymous=False)
    line_sanity_node = LineSanityNode()
    rospy.on_shutdown(line_sanity_node.onShutdown)
    rospy.spin()



