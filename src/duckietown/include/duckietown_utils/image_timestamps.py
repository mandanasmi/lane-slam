

from .image_composition import make_images_grid



def add_duckietown_header(img, log_name, time, frame):
    s = 'Duckietown %4d  %6.2f   %s' % (frame, time, log_name)
    return add_header_to_image(img, s)

def add_header_to_image(img, s):
    import cv2
    import numpy as np
    H, W = 35, img.shape[1]
    black = np.zeros((H, W, 3), 'uint8')
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(black,s,(10,25),font,0.75,(0,255,255),1,cv2.LINE_AA)  # @UndefinedVariable
    res = make_images_grid([black, img], cols=1)
    return res
