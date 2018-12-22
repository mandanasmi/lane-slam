#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
#from sensor_msgs.msg import CompressedImage,Image

import random


image_dir = '/home/mfe/'
image_name = 'test.png'

# Open image and split into 3 channels
img = cv2.imread( image_dir+image_name )

# cv2.imshow('image',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
r,g,b = cv2.split(img)

# crop image and only keep bottom third
img_height = len(r)
img_width = len(r[0])

print img_width,img_height
bottom_third_top_index = int(img_height*2/3.0)
# r_bottom = r[bottom_third_top_index:]
# b_bottom = b[bottom_third_top_index:]
# g_bottom = g[bottom_third_top_index:]

# Randomly sample numSampes^2 pixels from the image (uniformly distributed in x,y)
numSamples = 100
iList = [random.randint(bottom_third_top_index,len(r)-1) for i in range(numSamples)]
jList = [random.randint(0,img_width) for i in range(numSamples)]
r_rand = [r[i][j] for i in iList for j in jList]
b_rand = [b[i][j] for i in iList for j in jList]
g_rand = [g[i][j] for i in iList for j in jList]

# Plot rgb as xyz
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(r_rand, g_rand, b_rand)
ax.set_xlabel('R')
ax.set_ylabel('G')
ax.set_zlabel('B')
plt.show()


# #for topic, msg, t in bag.read_messages(topics=['/ferrari/camera_node/img_low/compressed']):
# for topic, msg, t in bag.read_messages(topics=['/rosberrypi_cam/image_raw']):

# 	np_arr = np.fromstring(msg.data, np.uint8)
# 	cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
# 	#adj_color = cv2.applyColorMap(cv_image, cv2.COLORMAP_AUTUMN)
# 	height = len(np_arr)
# 	horizon_height = int(height/2.0)
# 	img_slab = np_arr[horizon_height:height]
# 	# b,g,r = cv2.split(img)       # get b,g,r
# 	# rgb_img = cv2.merge([r[n:n+w],g[n:n+w],b[n:n+w]])     # switch it to rgb
# 	#img = Image.fromarray( np.asarray( np.clip(img,0,255), dtype="uint8"), "RGB" )
# 	img = Image.fromarray(img_slab)
# 	img.save( '/home/mfe/test2.png' )
# 	break
# bag.close()

# bag = rosbag.Bag('/home/mfe/Downloads/mercedes_dark2_2016-01-03-21-59-30_4.bag')