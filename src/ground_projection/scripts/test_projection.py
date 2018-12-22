#!/usr/bin/env python
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from ground_projection.srv import GetGroundCoord
from duckietown_msgs.msg import Vector2D

def call_service_get_ground_coordinate(req, veh):
  rospy.wait_for_service(veh + "/ground_projection/get_ground_coordinate")
  try:
    get_ground_coord = rospy.ServiceProxy(veh + '/ground_projection/get_ground_coordinate', GetGroundCoord)
    resp = get_ground_coord(req)
    return resp.gp
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e

def mouse_cb(event, x, y, flags, param):
  if event == cv2.EVENT_LBUTTONDOWN:
    w, h = param
    normalized_uv = Vector2D()
    normalized_uv.x = float(x)/float(w)
    normalized_uv.y = float(y)/float(h)
    
    gp = call_service_get_ground_coordinate(normalized_uv, veh)

    print "image coordinate: (%d, %d)" % (x, y)
    print "normalized image coordinate: (%f, %f)" % (normalized_uv.x, normalized_uv.y)
    print "ground coordinate: (%f, %f, %f)" % (gp.x, gp.y, gp.z)

def get_image_topic_name(veh):
  image_topic_name = veh + "/camera_node/image_rect"
  try:
    rospy.wait_for_message(image_topic_name, Image, timeout=3)
    return image_topic_name
  except rospy.ROSException, e:
    print "%s" % e

  image_topic_name = veh + "/camera_node/image/raw"
  try:
    rospy.wait_for_message(image_topic_name, Image, timeout=5)
    return image_topic_name
  except rospy.ROSException, e:
    print "%s" % e

  return None

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print "usage: " + sys.argv[0] + " vehicle name"
    sys.exit(1)
  
  param = sys.argv[1]
  param = param.replace('veh:=','')
  print('Using vehicle name %r.' % param)
  veh = "/" + param

  bridge = CvBridge()

  rospy.init_node("test_projection")
  print "please click the image to look up the ground plane coordinate"
  print "press 'ESC' key to exit"

  cv2.namedWindow("image")
  cv2.setMouseCallback("image", mouse_cb, param=(640, 480))
  key = 0
  
  image_topic_name = get_image_topic_name(veh)
  print "image topic name: " + image_topic_name

  if image_topic_name is not None:
    while(key is not 27):
      img = rospy.wait_for_message(image_topic_name, Image)
      gray = bridge.imgmsg_to_cv2(img, "mono8")
      cv2.imshow("image", gray)
      key = cv2.waitKey(0)
