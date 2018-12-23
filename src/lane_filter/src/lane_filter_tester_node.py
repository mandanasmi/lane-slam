#!/usr/bin/env python
import rospy 
from duckietown_msgs.msg import SegmentList, Segment

class LaneFilterTesterNode(object):
    def __init__(self):
        #node_name = "Lane Filter Tester"
        pub_fake_segment_list = rospy.Publisher("~segment_list", SegmentList, queue_size=1)
        rospy.sleep(1)

        seg = Segment()
        seg.points[0].x = rospy.get_param("~x1")
        seg.points[0].y = rospy.get_param("~y1")
        seg.points[1].x = rospy.get_param("~x2")
        seg.points[1].y = rospy.get_param("~y2")
        color = rospy.get_param("~color")


        if color=="white":
            seg.color=seg.WHITE
        elif color=="yellow":
            seg.color=seg.YELLOW
        elif color=="red":
            seg.color=seg.RED
        else:
            print "error no color specified"
        seg_list = SegmentList()
        seg_list.segments.append(seg)
        pub_fake_segment_list.publish(seg_list)


    def onShutdown(self):
        rospy.loginfo("[LaneFilterTesterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('lane_filter_tester',anonymous=False)
    lane_filter_tester_node = LaneFilterTesterNode()
    rospy.on_shutdown(lane_filter_tester_node.onShutdown)
