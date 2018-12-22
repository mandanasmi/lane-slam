#!/usr/bin/env python
import sys
import rospy
from ground_projection.srv import EstimateHomography
from sensor_msgs.msg import CameraInfo, Image
import numpy as np

def call_service_estimate_homography(req, veh):
    rospy.wait_for_service(veh + "/ground_projection/estimate_homography")
    try:
        service = rospy.ServiceProxy(veh + '/ground_projection/estimate_homography', 
                                     EstimateHomography)
        resp = service(req)
        return resp.homography
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)
        raise

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print "usage: " + sys.argv[0] + " vehicle name"
    sys.exit()

  veh = "/" + sys.argv[1]

  rospy.init_node("ex_estimate_homography")

  # get an image frame
  img = rospy.wait_for_message(veh + "/camera_node/image_rect", Image)
  print("got an image frame")

  H = call_service_estimate_homography(img, veh)
  
  print(H)
