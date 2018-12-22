import cv2

from duckietown_msgs.msg import (Segment)  # @UnresolvedImport
import numpy as np


BLACK = (0,0,0)
BGR_RED = (0,0,255)
BGR_GREEN = (0,255,0)
BGR_WHITE = (255,255,255)
BGR_YELLOW = (0, 255,255)

def vs_fancy_display(image_cv, segment_list):
    """
         
    """
    colors = {Segment.WHITE: BGR_WHITE,
              Segment.RED: BGR_RED,
              Segment.YELLOW: BGR_YELLOW}
    
    ground = np.copy(image_cv)
    shape = ground.shape[:2]
    
    ground = ground / 4 + 120
     
    for segment in segment_list.segments:
        
        p1 = segment.pixels_normalized[0]
        p2 = segment.pixels_normalized[1]
        
        P1 = normalized_to_image(p1, shape)
        P2 = normalized_to_image(p2, shape) 
        
        paint = colors[segment.color]
        width = 1
        cv2.line(ground, P1, P2, paint, width)
        
    return ground

def normalized_to_image(p,shape):
    x, y = p.x, p.y
    H, W = shape
    X = x * W
    Y = y * H
    return int(X), int(Y)

