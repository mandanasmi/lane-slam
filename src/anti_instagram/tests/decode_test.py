#!/usr/bin/env python

from anti_instagram import  logger
import timeit
from anti_instagram.utils import get_rospkg_root
import os
from duckietown_utils.jpg import image_cv_from_jpg, rgb_from_jpg_by_JPEG_library, rgb_from_jpg_by_PIL
 
def get_test_image():
    package_root = get_rospkg_root('anti_instagram')
    return os.path.join(package_root, 'tests', 'frame.jpg')

def setup():
    with open(get_test_image(), 'rb') as f:
        return f.read()

def decode_cv_orig(data):
    return image_cv_from_jpg(data)
#
# def decode_cv_buf(data):
#     return image_cv_from_jpg_buf(data)

def decode2(data):
    return rgb_from_jpg_by_PIL(data)

def decode3(data):
    return rgb_from_jpg_by_JPEG_library(data)

import numpy as np
def create_empty_image(data):
    r = np.zeros((480, 640, 3), np.uint8)
    return r
#     cv.CreateImage(cv.GetSize(src), cv.IPL_DEPTH_8U, 1).
    
def wrap(method, data):
    res = method(data)
    # print('%s returned %s' % (method.__name__, res.shape))

if __name__ == '__main__':
    n = 10
    import platform
    proc = platform.processor()

    methods = ['decode_cv_orig',  # 'decode_cv_buf',

                'decode2', 'decode3', 'create_empty_image']
    for m in methods:
        tn = timeit.timeit(stmt='from __main__ import %s, wrap; wrap(%s, data)' % (m, m),
                       setup='from __main__ import setup; data=setup()',
                       number=n
                       )
        t = tn / n
        logger.info("%s: method %s, avg over %d tries: %.1f ms " % (proc, m, n, t * 1000))

