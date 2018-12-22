#!/usr/bin/env python
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from ground_projection.srv import GetGroundCoord
from duckietown_msgs.msg import Vector2D
import numpy as np

class GroundProjectionTest:
  def __init__(self, veh):
    self.board_w = 7
    self.board_h = 5
    self.square_size = 0.031
    self.x_offset = 0.191
    self.y_offset = -0.093

    self.width = 640
    self.height = 480

    self.veh = veh
    self.image_topic_name = self.get_image_topic_name(veh)
    print "image topic name: " + self.image_topic_name

    self.th_mean_sq_dist = 0.03**2 # 3cm error bound

  def get_image_topic_name(self, veh):
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

  def test(self, corners):
    # make sure if order of corners is right
    corner_lr = corners[0][0] # lower-right point
    corner_ur = corners[(self.board_h-1)*(self.board_w)][0] # upper-right point
    corner_ul = corners[(self.board_h)*(self.board_w)-1][0] # upper-left point
    h_flipped = False
    v_flipped = False
    
    if corner_ul[0] > corner_ur[0]:
      h_flipped = True
    
    if corner_lr[1] < corner_ur[1]:
      v_flipped = True

    # prepare lists of image and ground point coordinates
    pts_img = []
    pts_gnd = []
    
    for r in range(self.board_h):
      for c in range(self.board_w):
        gnd_pt = np.array([
          float(r)*self.square_size + self.x_offset, 
          float(c)*self.square_size + self.y_offset
          ], dtype=np.float32)
        pts_gnd.append(gnd_pt)
        img_pt = corners[
          (self.board_h - 1 -r if v_flipped else r)*(self.board_w) + 
          (self.board_w - 1 -c if h_flipped else c)
        ]
        pts_img.append(img_pt[0])

    # get projected ground coordinates
    pts_gnd_est = []
    for img_pt in pts_img:
      gp = self.get_gnd_coord(img_pt[0], img_pt[1])
      gp_est = np.array([gp.x, gp.y, gp.z], dtype=np.float32)
      pts_gnd_est.append(gp_est)

    # estimate squared distance
    assert len(pts_gnd) is len(pts_gnd_est), "# of ground truth points is not equal to # of estimated points"
    sum_sq_dist = 0.
    for gt, est in zip(pts_gnd, pts_gnd_est):
      sq_dist = (gt[0]-est[0])**2 + (gt[1]-est[1])**2
      print "sq_dist: ", sq_dist
      sum_sq_dist += sq_dist
    mean_sq_dist = sum_sq_dist/len(pts_gnd)

    print "mean_sq_dist: ", mean_sq_dist
    print "self.th_mean_sq_dist: ", self.th_mean_sq_dist

    # decide if it passes or fails
    if mean_sq_dist < self.th_mean_sq_dist:
      return True
    else:
      return False

  def get_gnd_coord(self, x, y):
    normalized_uv = Vector2D()
    normalized_uv.x = float(x)/float(self.width)
    normalized_uv.y = float(y)/float(self.height)

    gp = self.call_service_get_ground_coordinate(normalized_uv, self.veh)

    return gp

  def run(self):
    img = rospy.wait_for_message(self.image_topic_name, Image)
    color = bridge.imgmsg_to_cv2(img, "bgr8")
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (7, 5), None)
    if ret:
      assert len(corners) is 35, "number of corners are wrong"
      return self.test(corners)
    else:
      rospy.logerr("cannot find all corners")
      rospy.logerr("make sure your camera is looking at the chessboard")
      return False

  def call_service_get_ground_coordinate(self, req, veh):
    rospy.wait_for_service(veh + "/ground_projection/get_ground_coordinate")
    try:
      get_ground_coord = rospy.ServiceProxy(veh + '/ground_projection/get_ground_coordinate', GetGroundCoord)
      resp = get_ground_coord(req)
      return resp.gp
    except rospy.ServiceException, e:
      print "Service call failed: %s" % e

if __name__ == "__main__":
  bridge = CvBridge()

  rospy.init_node("test_projection")

  veh = rospy.get_param("~veh", "porsche911")
  print('Using vehicle name %r.' % veh)
  veh = "/" + veh
  gpt = GroundProjectionTest(veh)

  try:
    pass_fail = gpt.run()
    if pass_fail:
      print "result: " + '\033[92m' + "passed" + '\033[0m'
    else:
      print "result: " + '\033[91m' + "failed" + '\033[0m'
  except KeyboardInterrupt:
    print "test shutting down"
