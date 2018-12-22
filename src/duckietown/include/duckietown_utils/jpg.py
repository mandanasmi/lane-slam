"""
    The many options to convert JPG into data. 
"""


import os

from PIL import ImageFile  # @UnresolvedImport

from duckietown_utils import logger

from .file_utils import write_data_to_file
from .image_composition import make_images_grid  # @UnusedImport


def jpg_from_image_cv(image):
    import cv2
    return cv2.imencode('.jpg', image)[1].tostring()
     

def image_cv_from_jpg(data):
    """ Returns an OpenCV BGR image from a string """
    import cv2
    import numpy as np
    s = np.fromstring(data, np.uint8)
    image_cv = cv2.imdecode(s, cv2.IMREAD_COLOR)
    if image_cv is None:
        msg = 'Could not decode image (cv2.imdecode returned None). '
        msg += 'This is usual a sign of data corruption.'
        raise ValueError(msg)
    return image_cv


def image_cv_from_jpg_fn(fn):
    """ Read a JPG from a file """
    if not os.path.exists(fn):
        msg = "File does not exist: %s" % fn
        raise ValueError(msg)
    with open(fn) as f:
        return image_cv_from_jpg(f.read())

def write_jpg_to_file(image_cv, fn):
    """ Assuming image_cv is a BGR image, write to the file fn. """
    data = jpg_from_image_cv(image_cv)
    write_data_to_file(data, fn)


# class Storage:
#     dst = None
# 
# def image_cv_from_jpg_buf(data):
#     pass
#     """ Returns an OpenCV BGR image from a string """
#     s = np.fromstring(data, np.uint8)
#     if Storage.dst is not None:
#         image_cv = cv2.imdecode(s, cv2.IMREAD_COLOR, dst=Storage.dst)
#     else:
#         image_cv = cv2.imdecode(s, cv2.IMREAD_COLOR)
#     Storage.dst = image_cv
#     return image_cv


# Second option: use PIL

def rgb_from_jpg_by_PIL(data):
    """ Warning: this returns RGB """
    import numpy as np
    parser = ImageFile.Parser()
    parser.feed(data)
    res = parser.close() 
    res = np.asarray(res)
    return res

# third option: jpeg library

def rgb_from_jpg_by_JPEG_library(data):
    import numpy as np
    try:
        import jpeg4py as jpeg
    except ImportError:
        installation = """
sudo apt-get install -y libturbojpeg  python-cffi
sudo pip install jpeg4py
"""
        logger.error(installation)
        raise

    jpg_data = np.fromstring(data, dtype=np.uint8)
    image_cv = jpeg.JPEG(jpg_data).decode()
    return image_cv


def image_clip_255(image_float):
    """ Clips to 0,255 and converts to uint8 """
    import numpy as np
    h,w,_ = image_float.shape
    res = np.zeros((h,w,3), dtype=np.uint8)
    np.clip(image_float, 0, 255, out=res)
    return res
    
#     
# def imgmsg_from_cv2(image_cv):
#     return 
#         self.corrected_image = self.bridge.cv2_to_imgmsg(corrected_image_cv2,"bgr8"



# with libjpeg-turbo
# Convert from uncompressed image message
# image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")







