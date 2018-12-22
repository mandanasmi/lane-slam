from . import logger
import numpy as np
import cv2

# Load an color image in grayscale
#cref = [50.0,50.0,50.0]
def refPatchScale(image_fname, cref):
	img_orig = cv2.imread(image_fname)
	h = img_orig.shape[0]
	w = img_orig.shape[1]

	c1 = 250
	r1 = 320
	img_patch = img_orig[c1:c1+50,r1:r1+100,:]
	hp = img_patch.shape[0]
	wp = img_patch.shape[1]

	# just for testing
	cv2.imwrite("img_patch.jpg",img_patch)

	# m_mean = np.zeros(3)
	m_mean = np.mean(np.reshape(img_patch,[wp*hp,3]),0)

	logger.info('m_mean: %s' % m_mean)

	mscale = np.array(cref) / np.array(m_mean)

	logger.info('mscale: %s' % mscale)

	# reshape to do fast multiply
	img_scale = np.reshape(img_orig,[h*w,3])
	img_scale = np.reshape(img_scale*mscale,[h,w,3])

	return img_scale


def refPatchShift(image_fname,cref):
	img_orig = cv2.imread(image_fname)
	h = img_orig.shape[0]
	w = img_orig.shape[1]

	c1 = 300
	r1 = 300
	img_patch = img_orig[c1:c1+100,r1:r1+100,:]
	hp = img_patch.shape[0]
	wp = img_patch.shape[1]

	# m_mean = np.zeros(3)
	m_mean = np.mean(np.reshape(img_patch,[wp*hp,3]),0)

	logger.info('m_mean: %s' % m_mean)

	mshift = np.array(cref) - np.array(m_mean)


	logger.info('mshift: %s' % mshift)

	img_shift = np.reshape(img_orig,[h*w,3])
	img_shift = np.reshape(img_shift+mshift,[h,w,3])

	return img_shift

