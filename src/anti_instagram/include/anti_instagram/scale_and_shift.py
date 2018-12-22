from . import logger
import numpy as np

class SASParams():
    algorithm = 2

def scaleandshift(img, scale, shift):
    """ Returns a float image, which might be outside of [0,255]"""
    #logger.info('scale: %s' % scale)
    #logger.info('shift: %s' % shift)

    assert img.shape[2] == 3
    assert len(scale) == 3, scale
    assert len(shift) == 3, shift

    if SASParams.algorithm == 1:
        res = scaleandshift1(img, scale, shift)
    elif SASParams.algorithm == 2:
        res = scaleandshift2(img, scale, shift)
    else:
        assert False

    return res

def scaleandshift2(img, scale, shift):
    img_shift = np.zeros(img.shape, dtype='float32')
    for i in range(3):
        s = np.array(scale[i]).astype('float32')
        p = np.array(shift[i]).astype('float32')
        np.multiply(img[:,:,i], s, out=img_shift[:, :, i])
        img_shift[:, :, i] += p

    return img_shift

def scaleandshift1(img, scale, shift):
    h = img.shape[0]
    w = img.shape[1]

    img_scale = np.reshape(img, [h * w, 3])
    img_scale = np.reshape(img_scale * np.array(scale), [h, w, 3])

    img_shift = np.reshape(img_scale, [h * w, 3])
    img_shift = np.reshape(img_shift + np.array(shift), [h, w, 3])

    return img_shift
