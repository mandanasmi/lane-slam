
def gray2rgb(gray):
    '''
        Converts a H x W grayscale into a H x W x 3 RGB image
        by replicating the gray channel over R,G,B.

        :param gray: grayscale
        :type  gray: array[HxW](uint8),H>0,W>0

        :return: A RGB image in shades of gray.
        :rtype: array[HxWx3](uint8)
    '''
#    assert_gray_image(gray, 'input to gray2rgb')
    import numpy as np
    rgb = np.zeros((gray.shape[0], gray.shape[1], 3), dtype='uint8')
    for i in range(3):
        rgb[:, :, i] = gray
    return rgb

def bgr_from_rgb(rgb):
    import cv2
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return bgr

def rgb_from_bgr(rgb):
    import cv2
    bgr = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    return bgr

def zoom_image(im, zoom = 4):
    s = (im.shape[1] * zoom, im.shape[0] * zoom)
    import cv2
    imz = cv2.resize(im, s, interpolation=cv2.INTER_NEAREST)
    return imz
