import cv2
from matplotlib import pyplot as plt

# borrowed from: http://docs.opencv.org/3.1.0/d1/db7/tutorial_py_histogram_begins.html#gsc.tab=0

import sys

img = cv2.imread(sys.argv[1])
color = ('b','g','r')
for i,col in enumerate(color):
	histr = cv2.calcHist([img],[i],None,[256],[0,256])
	plt.plot(histr,color = col)
	plt.xlim([0,256])
#plt.savefig()
plt.show()